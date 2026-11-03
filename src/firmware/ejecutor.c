// Es el "cerebro" de la ESP32, escrito en C portable para poder probarlo en el PC: recibe los
// planes de la Pi, elige cual vale en cada instante, interpola la velocidad entre nodos, limita
// la aceleracion, frena ante limites de recorrido o si se acaba el plan, y traduce todo a
// micropasos para los drivers. No sabe nada de pines ni de la UART: eso lo hace main.cpp.
// Nucleo del ejecutor (ver ejecutor.h).
#include "ejecutor.h"

#include <math.h>
#include <string.h>

// a es mas nuevo que b con numeros de secuencia de 16 bits que dan la vuelta.
// Truco de la resta con signo: con numeros que dan la vuelta (65535 -> 0), "a - b" visto como
// entero con signo de 16 bits es positivo si a esta "adelante" de b en menos de media vuelta.
static int mas_nuevo(uint16_t a, uint16_t b)
{
    return (int16_t)(uint16_t)(a - b) > 0;
}
//---------------------------------------------------------------------------
// Segundos desde t_a hasta tau (negativo si t_a es futuro), con aritmetica modular.
static double transcurrido(uint32_t tau, uint32_t t_a)
{
    return (int32_t)(tau - t_a) * 1e-6;
}
//---------------------------------------------------------------------------
// Arranca el nucleo en reposo en la pose q0, sin planes.
void nucleo_iniciar(NucleoEjecutor *e, const ConfigEjecutor *cfg, const double q0[2])
{
    memset(e, 0, sizeof(*e));
    e->cfg = *cfg;
    for (int j = 0; j < 2; ++j) {
        e->q_cmd[j] = q0[j];
        e->pasos[j] = (long)floor(q0[j] / cfg->micropaso[j] + 0.5);
    }
}
//---------------------------------------------------------------------------
// Guarda un plan recibido: se ignora si es mas viejo que el vigente o esta repetido; si la lista
// esta llena, se descarta el mas viejo.
void nucleo_agregar_plan(NucleoEjecutor *e, const MsgPlan *p)
{
    if (e->hay_vigente && !mas_nuevo(p->secuencia, e->secuencia))
        return; // viejo o repetido
    for (int i = 0; i < e->n_planes; ++i)
        if (e->planes[i].secuencia == p->secuencia)
            return;
    if (e->n_planes == EJ_PLANES) {
        // Se descarta el mas viejo.
        int v = 0;
        for (int i = 1; i < e->n_planes; ++i)
            if (mas_nuevo(e->planes[v].secuencia, e->planes[i].secuencia))
                v = i;
        e->planes[v] = e->planes[--e->n_planes];
    }
    e->planes[e->n_planes++] = *p;
}
//---------------------------------------------------------------------------
// Un mensaje que llego por la UART: si es un plan valido se guarda y se anota cuando llego.
int nucleo_recibir(NucleoEjecutor *e, const uint8_t *msg, int n, uint32_t tau)
{
    MsgPlan p;
    if (!msg_plan_decodificar(msg, n, &p))
        return 0;
    nucleo_agregar_plan(e, &p);
    e->tau2 = tau;
    return 1;
}
//---------------------------------------------------------------------------
// Velocidad que pide el plan para un eje en el instante tau: interpolacion lineal entre el
// nodo anterior y el siguiente (los nodos estan separados Ts). Antes de t_a vale el primer
// nodo; despues del ultimo nodo marca "agotado" para que el ciclo frene.
double nucleo_consigna_plan(const MsgPlan *p, double Ts, uint32_t tau, int eje, int *indice, int *agotado)
{
    const int ultimo = p->M + p->N_ff;
    const double x = transcurrido(tau, p->t_a) / Ts;
    *agotado = 0;
    if (x <= 0.0) {
        *indice = 0;
        return msg_int16_a_velocidad(p->nodos[0][eje]);
    }
    const int j = (int)floor(x);
    if (j >= ultimo) {
        *indice = ultimo;
        *agotado = 1;
        return msg_int16_a_velocidad(p->nodos[ultimo][eje]);
    }
    *indice = j;
    const double f = x - j;
    const double a = msg_int16_a_velocidad(p->nodos[j][eje]), b = msg_int16_a_velocidad(p->nodos[j + 1][eje]);
    return a + f * (b - a);
}
//---------------------------------------------------------------------------
// Seleccion: el plan valido mas nuevo con t_a <= tau pasa a ser el vigente; los que quedan
// atras se descartan.
static void seleccionar(NucleoEjecutor *e, uint32_t tau)
{
    int mejor = -1;
    for (int i = 0; i < e->n_planes; ++i)
        if (transcurrido(tau, e->planes[i].t_a) >= 0.0
            && (mejor < 0 || mas_nuevo(e->planes[i].secuencia, e->planes[mejor].secuencia)))
            mejor = i;
    if (mejor < 0)
        return;
    if (!e->hay_vigente || mas_nuevo(e->planes[mejor].secuencia, e->secuencia)) {
        e->vigente = e->planes[mejor];
        e->secuencia = e->vigente.secuencia;
        e->hay_vigente = 1;
    }
    int k = 0;
    for (int i = 0; i < e->n_planes; ++i)
        if (mas_nuevo(e->planes[i].secuencia, e->secuencia))
            e->planes[k++] = e->planes[i];
    e->n_planes = k;
}
//---------------------------------------------------------------------------
// Un ciclo de 1 ms: elige el plan vigente, calcula la velocidad de cada eje (plan, frenado o
// limite), le aplica los topes, actualiza el observador del error de posicion y deja en
// e->pasos la posicion objetivo en micropasos.
void nucleo_ciclo(NucleoEjecutor *e, uint32_t tau, const double q_enc[2])
{
    const ConfigEjecutor *c = &e->cfg;
    seleccionar(e, tau);
    e->banderas = e->hay_vigente ? 0 : TEL_SIN_PLAN;
    int indice = 0, agotado = 0;
    for (int j = 0; j < 2; ++j) {
        const double qd = e->qd_cmd[j];
        // Consigna del plan, o frenado si no hay plan, se agoto la cola o la Pi lo pide.
        double ref = 0.0;
        int frenar = !e->hay_vigente || (e->vigente.banderas & PLAN_MODO_SEGURO);
        if (e->hay_vigente && !frenar) {
            ref = nucleo_consigna_plan(&e->vigente, c->Ts, tau, j, &indice, &agotado);
            if (agotado) {
                frenar = 1;
                e->banderas |= TEL_FRENANDO;
            } else if (indice >= e->vigente.M)
                e->banderas |= TEL_EN_COLA;
        }
        if (frenar) {
            const double paso = c->a_frenado * c->dt;
            ref = fabs(qd) <= paso ? 0.0 : qd - (qd > 0 ? paso : -paso);
        }
        // Tope de velocidad sobre la consigna (no es un salto: no cuenta como limitador).
        if (ref > c->qd_max)
            ref = c->qd_max;
        if (ref < -c->qd_max)
            ref = -c->qd_max;
        // Limitador.
        double d = ref - qd;
        const double lim = c->a_lim * c->dt;
        if (fabs(d) > lim + 1e-12) {
            d = d > 0 ? lim : -lim;
            e->banderas |= TEL_LIMITADOR;
        }
        double qn = qd + d;
        // Limites de recorrido (ACT-4): si con a_frenado ya no alcanza a detenerse antes del
        // limite menos el margen, frena.
        const double hasta_max = c->q_max[j] - c->margen_limite - e->q_cmd[j];
        const double hasta_min = e->q_cmd[j] - (c->q_min[j] + c->margen_limite);
        const double paso_f = c->a_frenado * c->dt;
        if (qn > 0.0 && qn * c->dt + qn * qn / (2.0 * c->a_frenado) >= hasta_max) {
            qn = qd > paso_f ? qd - paso_f : 0.0;
            if (qn * c->dt > hasta_max)
                qn = hasta_max > 0.0 ? hasta_max / c->dt : 0.0;
            e->banderas |= TEL_LIMITE;
        } else if (qn < 0.0 && -qn * c->dt + qn * qn / (2.0 * c->a_frenado) >= hasta_min) {
            qn = qd < -paso_f ? qd + paso_f : 0.0;
            if (-qn * c->dt > hasta_min)
                qn = hasta_min > 0.0 ? -hasta_min / c->dt : 0.0;
            e->banderas |= TEL_LIMITE;
        }
        // Observador alfa-beta del angulo de carga, con el error medido antes de mover.
        const double e_med = e->q_cmd[j] - q_enc[j];
        const double e_pred = e->e_hat[j] + e->ed_hat[j] * c->dt;
        const double r = e_med - e_pred;
        e->e_hat[j] = e_pred + c->alfa * r;
        e->ed_hat[j] += c->beta * r / c->dt;
        // Trayectoria comandada (aceleracion constante en el ciclo: trapecio exacto) y DDA.
        e->q_cmd[j] += 0.5 * (qd + qn) * c->dt;
        e->qd_cmd[j] = qn;
        e->e_lento[j] += (e_med - e->e_lento[j]) * c->dt / (c->tau_correccion + c->dt);
        e->pasos[j] = (long)floor((e->q_cmd[j] + c->kp * e->e_lento[j]) / c->micropaso[j] + 0.5);
    }
    if (e->banderas & TEL_LIMITADOR)
        e->n_limitador++;
    // Fin de sesion: agotada la cola y con los dos ejes detenidos, se suelta el plan. Asi el
    // primero que llegue se acepta con cualquier secuencia (la Pi pudo reiniciarse y volver a
    // contar desde 0); por la UART no hay paquetes atrasados en segundos que puedan colarse.
    if (agotado && e->qd_cmd[0] == 0.0 && e->qd_cmd[1] == 0.0) {
        e->hay_vigente = 0;
        e->n_planes = 0;
        e->banderas = (uint8_t)((e->banderas & ~TEL_FRENANDO) | TEL_SIN_PLAN);
    }
    if (e->hay_vigente) {
        const double edad = transcurrido(tau, e->vigente.t_a) / c->Ts;
        e->edad = edad < 0.0 ? 0 : (edad > 255.0 ? 255 : (uint8_t)edad);
        e->indice = (uint8_t)(indice > 255 ? 255 : indice);
    }
}
//---------------------------------------------------------------------------
// Llena la telemetria con el estado actual (la ESP32 la envia cada 1/F_TELEMETRIA).
void nucleo_telemetria(const NucleoEjecutor *e, uint32_t tau, MsgTelemetria *t)
{
    t->tau = tau;
    for (int j = 0; j < 2; ++j) {
        t->q_hat[j] = (float)(e->q_cmd[j] - e->e_hat[j]);
        t->e_hat[j] = (float)e->e_hat[j];
    }
    t->secuencia = e->secuencia;
    t->indice = e->indice;
    t->edad = e->edad;
    t->banderas = e->banderas;
    t->tau2 = e->tau2;
    t->tau3 = tau;
}

// Predictor Phi (ver predictor.h).
#include "predictor.h"

#include <string.h>

// El predictor guarda una copia ("espejo") del nucleo de la ESP32 y los ultimos planes
// enviados, para saber donde estara la torreta cuando empiece a valer el proximo plan.
void phi_iniciar(Predictor *f, const ConfigEjecutor *cfg, const double q0[2])
{
    memset(f, 0, sizeof(*f));
    nucleo_iniciar(&f->espejo, cfg, q0);
}
//---------------------------------------------------------------------------
// Anota un plan recien enviado (se guarda en una lista circular de los ultimos).
void phi_enviado(Predictor *f, const MsgPlan *p)
{
    if (f->n_env == PHI_ENVIADOS) {
        memmove(f->enviados, f->enviados + 1, (PHI_ENVIADOS - 1) * sizeof(MsgPlan));
        f->n_env--;
    }
    f->enviados[f->n_env++] = *p;
}
//---------------------------------------------------------------------------
// Busca entre los enviados el plan con esa secuencia (NULL si ya no esta).
static const MsgPlan *buscar(const Predictor *f, uint16_t sec)
{
    for (int i = f->n_env - 1; i >= 0; --i)
        if (f->enviados[i].secuencia == sec)
            return &f->enviados[i];
    return NULL;
}
//---------------------------------------------------------------------------
// Avanza el espejo ciclo a ciclo desde su tick hasta `hasta` (inclusive), sin encoder: la
// trayectoria comandada no depende de el.
static void avanzar(NucleoEjecutor *e, uint32_t desde, uint32_t hasta)
{
    const uint32_t paso = (uint32_t)(e->cfg.dt * 1e6 + 0.5);
    for (uint32_t tau = desde + paso; (int32_t)(hasta - tau) >= 0; tau += paso) {
        const double q[2] = {e->q_cmd[0], e->q_cmd[1]};
        nucleo_ciclo(e, tau, q);
    }
}
//---------------------------------------------------------------------------
// Llega una telemetria: el espejo se pone igual al estado real que informa la ESP32.
void phi_telemetria(Predictor *f, const MsgTelemetria *t)
{
    NucleoEjecutor *e = &f->espejo;
    // El espejo se lleva hasta tau_tel con lo que creia comprometido, para tener q_dot_cmd por si
    // la ESP32 informa limitador o frenado.
    if (f->hay_tel)
        avanzar(e, f->tau_tel, t->tau);
    const double qd_espejo[2] = {e->qd_cmd[0], e->qd_cmd[1]};

    const MsgPlan *v = (t->banderas & TEL_SIN_PLAN) ? NULL : buscar(f, t->secuencia);
    e->hay_vigente = v != NULL;
    if (v) {
        e->vigente = *v;
        e->secuencia = v->secuencia;
    }
    e->n_planes = 0;
    for (int i = 0; i < f->n_env; ++i)
        if (!v || (int16_t)(uint16_t)(f->enviados[i].secuencia - v->secuencia) > 0)
            nucleo_agregar_plan(e, &f->enviados[i]);
    for (int j = 0; j < 2; ++j) {
        e->q_cmd[j] = (double)t->q_hat[j] + (double)t->e_hat[j];
        e->qd_cmd[j] = qd_espejo[j];
        if (v && !(t->banderas & (TEL_LIMITADOR | TEL_FRENANDO | TEL_LIMITE))) {
            int indice, agotado;
            e->qd_cmd[j] = nucleo_consigna_plan(v, e->cfg.Ts, t->tau, j, &indice, &agotado);
        } else if (!v)
            e->qd_cmd[j] = (t->banderas & TEL_SIN_PLAN) ? 0.0 : qd_espejo[j];
    }
    f->tau_tel = t->tau;
    f->hay_tel = 1;
}
//---------------------------------------------------------------------------
// Avanza una copia del espejo, ciclo a ciclo, hasta t_a y devuelve el estado predicho
// (posicion y velocidad de los dos ejes). Es la condicion inicial del NMPC.
int phi_predecir(const Predictor *f, uint32_t t_a, double x[4])
{
    NucleoEjecutor e = f->espejo;
    if (f->hay_tel)
        avanzar(&e, f->tau_tel, t_a);
    x[0] = e.q_cmd[0];
    x[1] = e.q_cmd[1];
    x[2] = e.qd_cmd[0];
    x[3] = e.qd_cmd[1];
    return f->hay_tel;
}

// En palabras simples: en cada cuadro de la camara este modulo (1) actualiza el EKF con la
// deteccion, (2) predice donde estara la torreta cuando el proximo plan empiece a valer (t_a),
// (3) le pide al planificador (el NMPC) las aceleraciones del horizonte, o busca si no hay
// blanco, y (4) arma el plan de velocidades que se envia a la ESP32. Tambien procesa la
// telemetria que vuelve de la ESP32 y lleva la cuenta de los planes perdidos.
// Controlador con aplicacion programada (ver ctl_programado.h).
#include "ctl_programado.h"

#include <math.h>
#include <string.h>

#include "quevedo.h"
#include "reloj_pc.h"
#include "trama.h"

// Configuracion del ejecutor de la ESP32 armada desde los parametros centrales: la Pi la
// necesita para su copia (espejo) del ejecutor.
ConfigEjecutor config_ejecutor(const Params *p)
{
    ConfigEjecutor c;
    c.Ts = p->lazo.Ts;
    c.dt = p->planta.dt_ejecutor;
    c.a_lim = p->ejecutor.a_lim;
    c.a_frenado = p->ejecutor.a_frenado;
    for (int j = 0; j < 2; ++j) {
        c.q_min[j] = p->lim.q_min[j];
        c.q_max[j] = p->lim.q_max[j];
    }
    c.margen_limite = p->ejecutor.margen_limite;
    c.qd_max = p->ejecutor.qd_max;
    for (int j = 0; j < 2; ++j)
        c.micropaso[j] = 2.0 * 3.14159265358979323846 / (200.0 * p->planta.micropasos[j] * p->planta.relacion[j]);
    c.kp = p->ejecutor.kp;
    c.tau_correccion = p->ejecutor.tau_correccion;
    c.alfa = p->ejecutor.alfa;
    c.beta = p->ejecutor.beta;
    return c;
}
//---------------------------------------------------------------------------

// Deja todo en su estado inicial: EKF, predictor, sincronizacion de relojes y monitor de
// perdidas, con la torreta en q0.
void ctl_programado_iniciar(CtlProgramado *c, const Params *m)
{
    memset(c, 0, sizeof(*c));
    c->geo.fu = m->cam.fu;
    c->geo.fv = m->cam.fv;
    c->geo.cu = m->cam.cu;
    c->geo.cv = m->cam.cv;
    c->geo.lx = m->camara.lx;
    c->alto = m->cam.height;
    c->Ts = m->lazo.Ts;
    c->delta_fb = m->com.delta_fb;
    c->M_pedido = m->com.M;
    c->M = m->com.M < PROG_N_MAX ? m->com.M : PROG_N_MAX;
    c->N_ff = m->com.N_ff;
    c->qd_max = m->ejecutor.qd_max;
    c->qdd_max = m->lim.qdd_max;
    c->mu_min = m->lim.q_min[1];
    c->mu_max = m->lim.q_max[1];
    c->tilt_activo = m->planta.tilt_activo;
    c->tilt_fijo = m->planta.tilt_fijo;
    c->sigma0 = m->camara.sigma0;
    c->sigma_ultima = m->camara.sigma0;
    ekf_iniciar(&c->ekf, &m->ekf, &c->geo);
    sync_iniciar(&c->sync, 10.0, 200e-6);
    const ConfigEjecutor ce = config_ejecutor(m);
    const double q0[2] = {0.0, 0.0};
    phi_iniciar(&c->phi, &ce, q0);
    ge_monitor_iniciar(&c->monitor);
    // gamma para el costo del NMPC (C12): error de posicion en pixeles, q_pos = f^2 W_s.
    double ell;
    if (!quevedo_eje(m->lazo.Ts, m->cam.fu * m->cam.fu * m->nmpc.W_s, 0.0, m->nmpc.W_u, &c->gamma, &ell))
        c->gamma = NAN;
}
//---------------------------------------------------------------------------
// Pose de la torreta en el instante t (reloj de la Pi), interpolada entre las telemetrias
// guardadas. El EKF la necesita en el instante exacto de captura de cada cuadro.
static void pose_en(const CtlProgramado *c, double t, double q[2])
{
    const long n = c->n_hist < PROG_HIST ? c->n_hist : PROG_HIST;
    const long ini = c->n_hist - n;
    long a = c->n_hist - 1;
    while (a > ini && c->h_t[a % PROG_HIST] > t)
        a--;
    const int ia = (int)(a % PROG_HIST);
    if (a == c->n_hist - 1 || c->h_t[ia] > t) {
        q[0] = c->h_q[ia][0];
        q[1] = c->h_q[ia][1];
        return;
    }
    const int ib = (int)((a + 1) % PROG_HIST);
    const double f = (t - c->h_t[ia]) / (c->h_t[ib] - c->h_t[ia]);
    for (int k = 0; k < 2; ++k)
        q[k] = c->h_q[ia][k] + f * (c->h_q[ib][k] - c->h_q[ia][k]);
}
//---------------------------------------------------------------------------
// Monitor de perdidas: un plan que nunca llego a estar vigente antes que uno posterior se
// perdio (o llego tarde y fue superado, que para el lazo es lo mismo).
static void registrar_vigente(CtlProgramado *c, uint16_t sec)
{
    if (!c->hay_vigente) {
        c->hay_vigente = 1;
        c->ultimo_vigente = sec;
        ge_monitor_registrar(&c->monitor, 0);
        return;
    }
    const int salto = (int16_t)(uint16_t)(sec - c->ultimo_vigente);
    if (salto <= 0)
        return;
    for (int i = 1; i < salto; ++i)
        ge_monitor_registrar(&c->monitor, 1);
    ge_monitor_registrar(&c->monitor, 0);
    c->ultimo_vigente = sec;
}
//---------------------------------------------------------------------------
// Una telemetria de la ESP32: se usa para sincronizar relojes, resincronizar el predictor,
// guardar la pose en el historial y detectar planes perdidos.
static void procesar_telemetria(CtlProgramado *c, const MsgTelemetria *t, double t4)
{
    // Estimacion inicial del desfase con la primera telemetria (sesgada en la latencia, que
    // los intercambios corrigen enseguida).
    if (!c->sync.valido) {
        c->sync.theta_us = t->tau3 - 1e6 * t4;
        c->sync.valido = 1;
    }
    // Intercambio de Cristian: tau2 es la llegada del ultimo plan; se empareja con el plan cuyo
    // t_a queda justo despues (un plan perdido no se empareja: su t_a quedaria fuera).
    if (t->tau2 != 0 && t->tau2 != c->tau2_usado) {
        for (int i = c->n_env - 1; i >= 0; --i) {
            const double adelanto = (int32_t)(c->env_ta[i] - t->tau2) * 1e-6;
            if (adelanto >= -2e-3 && adelanto <= c->delta_fb + 5e-3 && c->env_t1[i] > 0.0) {
                IntercambioSync x = {c->env_t1[i], (double)t->tau2, (double)t->tau3, t4};
                sync_agregar(&c->sync, &x);
                c->tau2_usado = t->tau2;
                break;
            }
        }
    }
    phi_telemetria(&c->phi, t);
    c->tau_tel = t->tau;
    c->hay_tel = 1;
    const int k = (int)(c->n_hist % PROG_HIST);
    c->h_t[k] = sync_t_de_tau(&c->sync, (double)t->tau);
    c->h_q[k][0] = t->q_hat[0];
    c->h_q[k][1] = t->q_hat[1];
    c->n_hist++;
    // Fuera de CTL_ACTUAR ningun plan se compromete: no hay perdidas que contar.
    if (!(t->banderas & TEL_SIN_PLAN) && c->actuacion == CTL_ACTUAR)
        registrar_vigente(c, t->secuencia);
}
//---------------------------------------------------------------------------
// Limita v al rango [-m, m].
static double recortar(double v, double m)
{
    return v > m ? m : (v < -m ? -m : v);
}
//---------------------------------------------------------------------------
// Busqueda: volver a la pose del ultimo avistamiento con el perfil de minimo tiempo
// q_dot_des = sign(e) min(q_dot_max, sqrt(2 q_ddot_max |e|), |e|/(2 T_s)), o detenerse si nunca se lo vio.
// El ultimo termino evita el ciclo limite del perfil discreto cerca del objetivo: sin el,
// un paso de T_s recorre mas que |e| y la consigna oscila a +-q_ddot_max.
static void buscar(const CtlProgramado *c, const double x0[4], double U[][2])
{
    double q[2] = {x0[0], x0[1]}, v[2] = {x0[2], x0[3]};
    for (int i = 0; i < PROG_N_MAX; ++i)
        for (int j = 0; j < 2; ++j) {
            const double e = c->hay_avistamiento ? c->q_avistamiento[j] - q[j] : 0.0;
            const double vd = (e < 0.0 ? -1.0 : 1.0)
                              * fmin(fmin(c->qd_max, sqrt(2.0 * c->qdd_max * fabs(e))), 0.5 * fabs(e) / c->Ts);
            U[i][j] = recortar(c->hay_avistamiento ? (vd - v[j]) / c->Ts : -v[j] / c->Ts, c->qdd_max);
            q[j] += c->Ts * v[j] + 0.5 * c->Ts * c->Ts * U[i][j];
            v[j] += c->Ts * U[i][j];
        }
}
//---------------------------------------------------------------------------
// El paso del controlador, una vez por cuadro. "ctx" es un puntero generico (void *) al
// estado del controlador: asi la interfaz Controlador sirve para cualquier implementacion.
static void paso(void *ctx, const EntradaCtl *in, SalidaCtl *out)
{
    CtlProgramado *c = ctx;
    const double t0 = reloj_pc_ms();
    out->n_bytes = 0;
    out->hay_prediccion = 0;
    if (in->t1_plan_anterior > 0.0 && c->n_env > 0)
        c->env_t1[c->n_env - 1] = in->t1_plan_anterior;
    for (int i = 0; i < in->n_msg; ++i) {
        MsgTelemetria t;
        if (msg_telemetria_decodificar(in->msg[i], MSG_TELEMETRIA_BYTES, &t))
            procesar_telemetria(c, &t, in->msg_t4[i]);
    }

    // EKF con la pose en el instante de captura (reloj de la Pi).
    const double t_sof = in->t_sof_us * 1e-6, t_fila = in->t_fila_us * 1e-6, t_e = in->t_e_us * 1e-6;
    out->nis = NAN;
    out->aceptada = 0;
    if (in->hay_z && c->n_hist > 0) {
        MedicionEKF m;
        m.t = t_sof + in->z[1] * t_fila + 0.5 * t_e;
        m.z[0] = in->z[0];
        m.z[1] = in->z[1];
        m.sigma = in->sigma;
        m.tamano = in->tamano;
        pose_en(c, m.t, m.q);
        out->aceptada = ekf_corregir(&c->ekf, &m, &out->nis);
        c->sigma_ultima = in->sigma;
        if (out->aceptada) {
            c->hay_avistamiento = 1;
            c->q_avistamiento[0] = m.q[0];
            c->q_avistamiento[1] = m.q[1];
        }
    } else
        ekf_sin_medicion(&c->ekf, t_sof + 0.5 * c->alto * t_fila + 0.5 * t_e);

    out->estado_qp = QP_SIN_RESOLVER;
    out->holgura = NAN;
    out->U[0] = out->U[1] = 0.0;
    out->ge_p = ge_monitor_p(&c->monitor);
    out->margen = ge_monitor_margen(&c->monitor, c->gamma);
    if (c->actuacion != CTL_ACTUAR)
        out->ge_p = out->margen = NAN;
    if (!c->hay_tel || c->actuacion == CTL_SOLO_ESTIMAR) {
        out->ms = reloj_pc_ms() - t0;
        return; // sin telemetria no hay base de tiempo ni estado del ejecutor
    }

    // t_a = t_s + Delta_fb, en ticks de la ESP32 y sobre la grilla de sus ciclos.
    const double t_a_pi = in->t_s_pi + c->delta_fb;
    const double paso_us = c->phi.espejo.cfg.dt * 1e6;
    const double bruto = sync_tau_de_t(&c->sync, t_a_pi);
    const double k_ciclos = ceil(((double)(int32_t)((uint32_t)(int64_t)llround(bruto) - c->tau_tel)) / paso_us);
    const uint32_t tau_a = c->tau_tel + (uint32_t)(int64_t)(k_ciclos * paso_us);
    const double t_a = sync_t_de_tau(&c->sync, (double)tau_a);
    double x0[4];
    phi_predecir(&c->phi, tau_a, x0);
    if (!c->tilt_activo) { // tilt sin motor: su pose es la fija, aunque la telemetria diga otra
        x0[1] = c->tilt_fijo;
        x0[3] = 0.0;
    }
    out->hay_prediccion = 1;
    out->tau_a = tau_a;
    memcpy(out->x_ta, x0, sizeof(x0));

    // Secuencia de aceleraciones: la del planificador si hay blanco; si no, buscar.
    double U[PROG_N_MAX][2];
    int buscando = !c->ekf.inicializado;
    if (c->ekf.inicializado && c->plan) {
        // NMPC: aceleracion del ultimo tramo comprometido (pendiente de q_dot_cmd en t_a, segun Phi)
        // y endurecimiento de percepcion kappa = max(1, (sigma_k/sigma0)^2).
        double xa[4];
        phi_predecir(&c->phi, tau_a - (uint32_t)paso_us, xa);
        const double u_prev[2] = {(x0[2] - xa[2]) / (paso_us * 1e-6), (x0[3] - xa[3]) / (paso_us * 1e-6)};
        const double r = c->sigma_ultima / c->sigma0, kappa = r * r > 1.0 ? r * r : 1.0;
        if (!c->ekf_estaba && c->plan_reiniciar)
            c->plan_reiniciar(c->plan_ctx);
        double hol = 0.0;
        for (int i = 0; i < PROG_N_MAX; ++i)
            U[i][0] = U[i][1] = 0.0;
        const int e = c->plan(c->plan_ctx, &c->ekf, t_a, x0, u_prev, kappa, U, PROG_N_MAX, &hol, &out->qp_iter,
                              &out->ms_qp);
        out->estado_qp = e == 0 ? QP_RESUELTO : QP_MAX_ITER;
        out->holgura = hol;
        if (e == PLAN_SIN_BLANCO) {
            buscando = 1;
            out->estado_qp = QP_SIN_RESOLVER;
            buscar(c, x0, U);
        }
    } else
        buscar(c, x0, U);
    if (!c->tilt_activo)
        for (int i = 0; i < PROG_N_MAX; ++i)
            U[i][1] = 0.0;
    c->ekf_estaba = c->ekf.inicializado;
    out->U[0] = U[0][0];
    out->U[1] = U[0][1];

    // Plan: nodo 0 en t_a, M nodos optimos y N_ff de cola con la velocidad de mirada.
    MsgPlan plan;
    memset(&plan, 0, sizeof(plan));
    plan.secuencia = ++c->secuencia;
    plan.t_a = tau_a;
    plan.M = (uint8_t)c->M;
    plan.N_ff = (uint8_t)c->N_ff;
    double v[2] = {x0[2], x0[3]};
    for (int j = 0; j < 2; ++j)
        plan.nodos[0][j] = msg_velocidad_a_int16(v[j]);
    for (int i = 0; i < c->M; ++i)
        for (int j = 0; j < 2; ++j) {
            v[j] = recortar(v[j] + U[i][j] * c->Ts, c->qd_max);
            plan.nodos[i + 1][j] = msg_velocidad_a_int16(v[j]);
        }
    // La cola extrapola ~0.9 s: la elevacion se satura en el rango del tilt como en el NMPC
    // (pasada la vertical, la mirada cambia de rama y el tilt invertia el signo), y una
    // velocidad no finita (EKF divergido, o el jacobiano de la mirada con |cos mu| < rho l_x) deja
    // la cola en reposo.
    for (int i = 1; i <= c->N_ff; ++i) {
        double qs[2] = {0.0, 0.0};
        if (!buscando) {
            double xi[EKF_N], P[EKF_N * EKF_N], dg[4];
            ekf_prediccion(&c->ekf, t_a + (c->M + i) * c->Ts, xi, P);
            ekf_saturar_elevacion(xi, c->mu_min, c->mu_max);
            geo_mirada_jac(&c->geo, xi, xi[6], dg);
            qs[0] = dg[0] * xi[2] + dg[1] * xi[3];
            qs[1] = dg[2] * xi[2] + dg[3] * xi[3];
            if (!isfinite(qs[0]) || !isfinite(qs[1]))
                qs[0] = qs[1] = 0.0;
            if (!c->tilt_activo)
                qs[1] = 0.0;
        }
        for (int j = 0; j < 2; ++j)
            plan.nodos[c->M + i][j] = msg_velocidad_a_int16(recortar(qs[j], c->qd_max));
    }
    uint8_t msg[MSG_PLAN_MAX_BYTES];
    const int n = msg_plan_codificar(&plan, msg);
    out->n_bytes = trama_codificar(msg, n, out->bytes);
    out->tipo = PLAN_VELOCIDAD;
    if (c->actuacion == CTL_SOMBRA) {
        out->ms = reloj_pc_ms() - t0;
        return; // el plan solo informa su tamano: Phi sigue sin planes comprometidos
    }
    phi_enviado(&c->phi, &plan);
    if (c->n_env == PROG_ENVIADOS) {
        memmove(c->env_sec, c->env_sec + 1, (PROG_ENVIADOS - 1) * sizeof(c->env_sec[0]));
        memmove(c->env_ta, c->env_ta + 1, (PROG_ENVIADOS - 1) * sizeof(c->env_ta[0]));
        memmove(c->env_t1, c->env_t1 + 1, (PROG_ENVIADOS - 1) * sizeof(c->env_t1[0]));
        c->n_env--;
    }
    c->env_sec[c->n_env] = plan.secuencia;
    c->env_ta[c->n_env] = tau_a;
    c->env_t1[c->n_env] = 0.0; // se completa con el t1 real en el ciclo siguiente
    c->n_env++;
    out->ms = reloj_pc_ms() - t0;
}
//---------------------------------------------------------------------------
// Registra el planificador (el NMPC) mediante punteros a funcion: el controlador llama a
// "plan" sin saber que hay detras, y el NMPC puede faltar si se compila sin acados.
void ctl_programado_usar_planificador(CtlProgramado *c, PlanificadorExterno plan, void (*reiniciar)(void *),
                                      void *ctx)
{
    c->plan = plan;
    c->plan_reiniciar = reiniciar;
    c->plan_ctx = ctx;
    c->M = c->M_pedido < PROG_N_MAX ? c->M_pedido : PROG_N_MAX;
}
//---------------------------------------------------------------------------
// Cambia lo que se hace con el plan: actuar (nivel 4), solo calcularlo (sombra, nivel 3) o ni
// siquiera eso (solo estimar, niveles 0 a 2).
void ctl_programado_fijar_actuacion(CtlProgramado *c, ActuacionCtl a)
{
    c->actuacion = a;
}
//---------------------------------------------------------------------------
// Pose interpolada en t (para el dashboard y las pruebas); 0 si todavia no hay telemetria.
int ctl_programado_pose(const CtlProgramado *c, double t, double q[2])
{
    if (c->n_hist <= 0)
        return 0;
    pose_en(c, t, q);
    return 1;
}
//---------------------------------------------------------------------------
// Empaqueta el controlador en la interfaz generica Controlador (puntero al estado + funcion
// paso), que es lo que usa la cadena del modo vivo.
Controlador ctl_programado_interfaz(CtlProgramado *c)
{
    Controlador k;
    k.ctx = c;
    k.paso = paso;
    k.usa_firmware = 1;
    return k;
}

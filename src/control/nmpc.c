// En palabras simples: el NMPC (control predictivo no lineal) mira hacia adelante un horizonte
// de N pasos, predice con el EKF por donde ira el blanco y busca las aceleraciones de los ejes
// que mantienen al blanco centrado en la imagen con el menor esfuerzo, sin salirse del campo
// de vision ni de los limites. El problema de optimizacion lo resuelve codigo generado por
// acados; aqui se le cargan los datos de cada ciclo y se lee la solucion. "RTI" significa
// que en cada ciclo se hace una sola iteracion, partiendo de la solucion del ciclo anterior.
// NMPC SQP-RTI (ver nmpc.h).
#include "nmpc.h"

#include <math.h>
#include <string.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_nmpc.h"
_Static_assert(NMPC_HORIZONTE == NMPC_N, "NMPC_HORIZONTE (nmpc.h) distinto de nmpc_solver.N (torreta.ini)");

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N NMPC_HORIZONTE
#define NPAR NMPC_NP
#define NY 8
#define U_TILT_FIJO 1e-6 // cota de u_tilt con el tilt sin motor (rad/s^2): ver nmpc_iniciar

#if NMPC_N != NMPC_HORIZONTE
#error "El horizonte del codigo generado no coincide con NMPC_HORIZONTE"
#endif

// Inversa de la normal estandar (aproximacion racional de Acklam, error relativo < 1.2e-9).
// Inversa de la distribucion normal: para que probabilidad p, cuantas desviaciones estandar.
// Se usa para los margenes de seguridad de la FOV con un nivel de confianza dado.
static double inv_normal(double p)
{
    static const double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
                                1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
    static const double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
                                6.680131188771972e+01, -1.328068155288572e+01};
    static const double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
                                -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
    static const double d[4] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                                3.754408661907416e+00};
    if (p < 0.02425) {
        const double q = sqrt(-2 * log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
               / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
    }
    if (p > 1 - 0.02425)
        return -inv_normal(1 - p);
    const double q = p - 0.5, r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q
           / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
}
//---------------------------------------------------------------------------
// Opciones del QP de HPIPM que se cambian sin regenerar el solver (fase 6, H32). Solo se tocan
// las que difieren del codigo generado, para que la configuracion por defecto sea identica.
// El modo va primero porque reinicia los demas argumentos de HPIPM, que se reponen con los
// valores del codigo generado (t0_init = 2 y Riccati de raiz cuadrada).
static int opciones_qp(nmpc_solver_capsule *c, const ParamsNmpc *p)
{
    static const char *const modos[QP_MODO_N] = {"BALANCE", "SPEED", "SPEED_ABS", "ROBUST"};
    ocp_nlp_config *cfg = nmpc_acados_get_nlp_config(c);
    void *opts = nmpc_acados_get_nlp_opts(c);
    if (p->qp_cond_N != N && nmpc_acados_update_qp_solver_cond_N(c, p->qp_cond_N) != 0)
        return 0;
    if (p->qp_modo != QP_MODO_BALANCE && p->qp_modo >= 0 && p->qp_modo < QP_MODO_N) {
        int t0 = 2, ric = 1;
        ocp_nlp_solver_opts_set(cfg, opts, "qp_hpipm_mode", (void *)modos[p->qp_modo]);
        ocp_nlp_solver_opts_set(cfg, opts, "qp_t0_init", &t0);
        ocp_nlp_solver_opts_set(cfg, opts, "qp_cond_ric_alg", &ric);
        ocp_nlp_solver_opts_set(cfg, opts, "qp_ric_alg", &ric);
    }
    int iter_max = p->qp_iter_max, caliente = p->qp_arranque_caliente;
    ocp_nlp_solver_opts_set(cfg, opts, "qp_iter_max", &iter_max);
    if (caliente)
        ocp_nlp_solver_opts_set(cfg, opts, "qp_warm_start", &caliente);
    return 1;
}
//---------------------------------------------------------------------------
// Crea el solver de acados, le aplica las opciones del .ini y prepara los pesos. Devuelve 0 si
// no se pudo crear.
int nmpc_iniciar(Nmpc *n, const Params *m)
{
    memset(n, 0, sizeof(*n));
    n->cfg = m->nmpc;
    n->geo.fu = m->cam.fu;
    n->geo.fv = m->cam.fv;
    n->geo.cu = m->cam.cu;
    n->geo.cv = m->cam.cv;
    n->geo.lx = m->camara.lx;
    n->Ts = m->lazo.Ts;
    n->sigma0 = m->camara.sigma0;
    n->t_e = m->camara.t_e;
    n->sigma_q = m->ekf.sigma_q;
    // La FOV se impone en coordenadas de rumbo (codegen/nmpc.py), en las que un punto a theta del
    // eje optico queda a f sen theta en vez de f tan theta: el radio r_fov (px) pasa a
    // r_fov cos(atan(r_fov/f)).
    const double f = sqrt(m->cam.fu * m->cam.fv);
    n->r_fov = m->lim.r_fov / sqrt(1.0 + (m->lim.r_fov / f) * (m->lim.r_fov / f));
    n->sigma_fov2 = (m->lim.r_fov / f) * (m->lim.r_fov / f);
    n->mu_min = m->lim.q_min[1];
    n->mu_max = m->lim.q_max[1];
    n->tilt_activo = m->planta.tilt_activo;
    n->tilt_fijo = m->planta.tilt_fijo;
    n->beta_fov = inv_normal(1.0 - m->nmpc.eps_fov);
    n->Jp = m->planta.J[0];
    n->Jt = m->planta.J[1];
    n->b = m->planta.b[0];
    n->tau_c = m->planta.tau_c[0];
    n->mgl = m->planta.mgl;
    for (int j = 0; j < 2; ++j)
        n->tau_max[j] = m->nmpc.eta * m->planta.Km_Ip * m->planta.relacion[j];
    nmpc_solver_capsule *c = nmpc_acados_create_capsule();
    if (!c || nmpc_acados_create(c) != 0)
        return 0;
    n->capsula = c;
    if (!opciones_qp(c, &m->nmpc))
        return 0;
    // Cotas de q, q_dot (etapas 1..N) y de u.
    ocp_nlp_config *cfg = nmpc_acados_get_nlp_config(c);
    ocp_nlp_dims *dims = nmpc_acados_get_nlp_dims(c);
    ocp_nlp_in *in = nmpc_acados_get_nlp_in(c);
    ocp_nlp_out *out = nmpc_acados_get_nlp_out(c);
    const double lbx[4] = {m->lim.q_min[0], m->lim.q_min[1], -m->ejecutor.qd_max, -m->ejecutor.qd_max};
    const double ubx[4] = {m->lim.q_max[0], m->lim.q_max[1], m->ejecutor.qd_max, m->ejecutor.qd_max};
    double lbu[2] = {-m->nmpc.u_max, -m->nmpc.u_max}, ubu[2] = {m->nmpc.u_max, m->nmpc.u_max};
    // Tilt sin motor: u_tilt en +-U_TILT_FIJO y, como x0 lo trae en
    // reposo en tilt_fijo, la dinamica lo deja ahi en todo el horizonte; el pan se optimiza
    // sabiendo que el tilt no se mueve. Con una cota de ancho cero el QP de puntos interiores no
    // tiene interior y HPIPM falla; el residuo de la banda no llega a la ESP32, porque el
    // controlador manda q_dot_tilt = 0.
    if (!m->planta.tilt_activo) {
        lbu[1] = -U_TILT_FIJO;
        ubu[1] = U_TILT_FIJO;
    }
    for (int i = 1; i <= N; ++i) {
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "lbx", (void *)lbx);
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "ubx", (void *)ubx);
    }
    for (int i = 0; i < N; ++i) {
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "lbu", (void *)lbu);
        ocp_nlp_constraints_model_set(cfg, dims, in, out, i, "ubu", (void *)ubu);
    }
    return 1;
}
//---------------------------------------------------------------------------
// Libera el solver.
void nmpc_liberar(Nmpc *n)
{
    if (n->capsula) {
        nmpc_acados_free((nmpc_solver_capsule *)n->capsula);
        nmpc_acados_free_capsule((nmpc_solver_capsule *)n->capsula);
        n->capsula = NULL;
    }
}
//---------------------------------------------------------------------------
// Olvida la solucion anterior: el proximo ciclo arranca desde la referencia (reenganche).
void nmpc_reiniciar(Nmpc *n)
{
    n->hay_solucion = 0;
}
//---------------------------------------------------------------------------
// Costo terminal: DARE del doble integrador de dos ejes, A = [I T I; 0 I], B = [T^2/2 I; T I],
// con Q = blkdiag(Qq, eps I) y R = W_u I. Iteracion en forma de Joseph,
//   K = (R + B^TPB)^-1 B^TPA,  P' = (A - BK)^T P (A - BK) + Q + K^T R K,
// simetrizada en cada paso: la forma clasica, A^TPA + Q - A^TPB (R + B^TPB)^-1 B^TPA, pierde la
// simetria y diverge con R chico frente a B^TPB (W_u = 1e-2 frente a ~10^5).
static void riccati(double Ts, const double Qq[4], double eps, double Wu, double P[16])
{
    double A[16] = {0}, B[8] = {0}, Q[16] = {0};
    for (int i = 0; i < 4; ++i)
        A[5 * i] = 1.0;
    A[0 * 4 + 2] = A[1 * 4 + 3] = Ts;
    B[0 * 2 + 0] = B[1 * 2 + 1] = 0.5 * Ts * Ts;
    B[2 * 2 + 0] = B[3 * 2 + 1] = Ts;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            Q[i * 4 + j] = Qq[2 * i + j];
    Q[2 * 4 + 2] = Q[3 * 4 + 3] = eps;
    memcpy(P, Q, sizeof(Q));
    for (int it = 0; it < 1000; ++it) {
        double PA[16], PB[8], S[4], BtPA[8], K[8], Acl[16], T[16], Pn[16];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += P[i * 4 + k] * A[k * 4 + j];
                PA[i * 4 + j] = s;
            }
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += P[i * 4 + k] * B[k * 2 + j];
                PB[i * 2 + j] = s;
            }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                double s = i == j ? Wu : 0.0;
                for (int k = 0; k < 4; ++k)
                    s += B[k * 2 + i] * PB[k * 2 + j];
                S[i * 2 + j] = s;
            }
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += B[k * 2 + i] * PA[k * 4 + j];
                BtPA[i * 4 + j] = s;
            }
        }
        const double det = S[0] * S[3] - S[1] * S[2];
        const double Si[4] = {S[3] / det, -S[1] / det, -S[2] / det, S[0] / det};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 4; ++j)
                K[i * 4 + j] = Si[i * 2] * BtPA[j] + Si[i * 2 + 1] * BtPA[4 + j];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                Acl[i * 4 + j] = A[i * 4 + j] - B[i * 2] * K[j] - B[i * 2 + 1] * K[4 + j];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += P[i * 4 + k] * Acl[k * 4 + j];
                T[i * 4 + j] = s;
            }
        double cambio = 0.0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = Q[i * 4 + j] + Wu * (K[i] * K[j] + K[4 + i] * K[4 + j]);
                for (int k = 0; k < 4; ++k)
                    s += Acl[k * 4 + i] * T[k * 4 + j];
                Pn[i * 4 + j] = s;
            }
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                const double v = 0.5 * (Pn[i * 4 + j] + Pn[j * 4 + i]);
                cambio = fmax(cambio, fabs(v - P[i * 4 + j]) / (fabs(P[i * 4 + j]) + 1e-12));
                P[i * 4 + j] = v;
            }
        if (cambio < 1e-12)
            break;
    }
}
//---------------------------------------------------------------------------
// Limita v al rango [lo, hi].
static double recortar(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
//---------------------------------------------------------------------------
// Plan de detencion (como el controlador programado sin blanco): frenar con la mayor
// aceleracion permitida. La proxima solucion parte de la referencia.
static void detenerse(Nmpc *n, const double x0[4], double U[][2])
{
    double v[2] = {x0[2], x0[3]};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < 2; ++j) {
            n->U[i][j] = recortar(-v[j] / n->Ts, -n->cfg.u_max, n->cfg.u_max);
            v[j] += n->U[i][j] * n->Ts;
            U[i][j] = n->U[i][j];
        }
    n->hay_solucion = 0;
}
//---------------------------------------------------------------------------
// Un ciclo del NMPC: arma la referencia con la prediccion del EKF, calcula los margenes de la
// FOV y el costo terminal, resuelve una iteracion y devuelve en U las aceleraciones del
// horizonte. Si el solver falla, usa la solucion anterior desplazada. Devuelve el estado del QP.
int nmpc_resolver(Nmpc *n, const EKFPieza *ekf, double t_a, const double x0[4], const double u_prev[2],
                  double kappa, double U[][2], double *holgura)
{
    nmpc_solver_capsule *cap = (nmpc_solver_capsule *)n->capsula;
    ocp_nlp_config *cfg = nmpc_acados_get_nlp_config(cap);
    ocp_nlp_dims *dims = nmpc_acados_get_nlp_dims(cap);
    ocp_nlp_in *in = nmpc_acados_get_nlp_in(cap);
    ocp_nlp_out *out = nmpc_acados_get_nlp_out(cap);
    ocp_nlp_solver *sol = nmpc_acados_get_nlp_solver(cap);
    const double Ts = n->Ts;

    // 1. Blanco predicho y referencias (fuera del solver), con dos canales de confianza. La
    // covarianza no se toca: los retranqueos la usan.
    //  - Aceleracion: psi_ddot_hat se encoge por su relacion senal a ruido,
    //    gamma_a = ||psi_ddot_hat||^2 / (||psi_ddot_hat||^2 + tr P_psi_ddot), para no perseguir aceleraciones mal estimadas al
    //    empezar y terminar las maniobras.
    //  - Velocidad (confianza_v): si tr P_psi(t_a) > sigma_FOV^2, con sigma_FOV = r_fov/f el semiangulo
    //    de la FOV, ni siquiera psi_hat(t_a) dice donde esta el blanco (deriva sin mediciones): se
    //    devuelve NMPC_SIN_BLANCO, con un plan de detencion, y el controlador busca. Si no,
    //    psi_dot_hat se encoge con gamma_v = ||psi_dot_hat||^2 / (||psi_dot_hat||^2 + tr P_psi_dot) (confianza_v = 2, por defecto), como
    //    la aceleracion, o con gamma_v = sigma_FOV^2 / (sigma_FOV^2 + tr P_psi(t_a)) (confianza_v = 1). La
    //    primera es la mejor medida en los laser (fase 6, registro): no persigue el ruido de
    //    psi_dot_hat al empezar y terminar los barridos lentos.
    EKFPieza prediccion = *ekf;
    if (n->cfg.confianza_v) {
        double xa[EKF_N], Pa[EKF_N * EKF_N];
        ekf_prediccion(ekf, t_a, xa, Pa);
        const double v = Pa[0] + Pa[EKF_N + 1];
        if (v > n->sigma_fov2) {
            detenerse(n, x0, U);
            *holgura = 0.0;
            n->ms = 0.0;
            return NMPC_SIN_BLANCO;
        }
        double g = n->sigma_fov2 / (n->sigma_fov2 + v);
        if (n->cfg.confianza_v == 2) {
            const double e = prediccion.xi[2] * prediccion.xi[2] + prediccion.xi[3] * prediccion.xi[3];
            const double w = prediccion.P[2 * EKF_N + 2] + prediccion.P[3 * EKF_N + 3];
            g = e + w > 0.0 ? e / (e + w) : 0.0;
        }
        ekf_encoger(&prediccion, 2, g);
    }
    if (n->cfg.confianza_a) {
        const double e = prediccion.xi[4] * prediccion.xi[4] + prediccion.xi[5] * prediccion.xi[5];
        const double v = prediccion.P[4 * EKF_N + 4] + prediccion.P[5 * EKF_N + 5];
        const double g = e + v > 0.0 ? e / (e + v) : 0.0;
        ekf_encoger(&prediccion, 4, g);
    }
    ekf = &prediccion;
    double psi[N + 1][2], psid[N + 1][2], rho = 0.0, Sig[N + 1][EKF_N * EKF_N];
    double qs[N + 1][2], qds[N + 1][2], us[N][2];
    for (int i = 0; i <= N; ++i) {
        double xi[EKF_N], dg[4];
        ekf_prediccion(ekf, t_a + i * Ts, xi, Sig[i]);
        // La elevacion extrapolada no sale del rango del tilt: pasada la vertical, el tilt de
        // referencia saltaba al otro lado y el NMPC alternaba entre +-u_max persiguiendolo.
        ekf_saturar_elevacion(xi, n->mu_min, n->mu_max);
        psi[i][0] = xi[0];
        psi[i][1] = xi[1];
        psid[i][0] = xi[2];
        psid[i][1] = xi[3];
        rho = xi[6];
        geo_mirada(&n->geo, psi[i], rho, qs[i]);
        geo_mirada_jac(&n->geo, psi[i], rho, dg);
        qds[i][0] = dg[0] * psid[i][0] + dg[1] * psid[i][1];
        qds[i][1] = dg[2] * psid[i][0] + dg[3] * psid[i][1];
    }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < 2; ++j)
            us[i][j] = recortar((qds[i + 1][j] - qds[i][j]) / Ts, -n->cfg.u_max, n->cfg.u_max);

    // Punto de linealizacion: la solucion anterior desplazada los d nodos que pasaron desde ella
    // (1 a 30 fps; 2 con la camara USB a 15 fps o tras perder un cuadro), con u*_i en las etapas
    // que quedan sin solucion; sin solucion anterior, la referencia.
    double Xl[N + 1][6], Ul[N][2];
    if (n->hay_solucion) {
        const long r = lround((t_a - n->t_sol) / Ts);
        const int d = r < 0 ? 0 : r > N ? N : (int)r;
        for (int i = 0; i + d <= N; ++i)
            memcpy(Xl[i], n->X[i + d], sizeof(Xl[i]));
        for (int i = 0; i < N; ++i)
            memcpy(Ul[i], i + d < N ? n->U[i + d] : us[i], sizeof(Ul[i]));
        for (int i = N - d + 1; i <= N; ++i)
            for (int j = 0; j < 2; ++j) {
                Xl[i][j] = Xl[i - 1][j] + Ts * Xl[i - 1][2 + j] + 0.5 * Ts * Ts * Ul[i - 1][j];
                Xl[i][2 + j] = Xl[i - 1][2 + j] + Ts * Ul[i - 1][j];
                Xl[i][4 + j] = Ul[i - 1][j];
            }
    } else {
        for (int i = 0; i <= N; ++i)
            for (int j = 0; j < 2; ++j) {
                Xl[i][j] = qs[i][j];
                Xl[i][2 + j] = recortar(qds[i][j], -2.0, 2.0);
                Xl[i][4 + j] = i ? us[i - 1][j] : u_prev[j];
            }
        for (int i = 0; i < N; ++i)
            memcpy(Ul[i], us[i], sizeof(Ul[i]));
    }
    for (int j = 0; j < 4; ++j)
        Xl[0][j] = x0[j];
    Xl[0][4] = u_prev[0];
    Xl[0][5] = u_prev[1];
    // Tilt sin motor: el punto de linealizacion lo deja fijo, como la solucion (si parte de la
    // referencia, el tilt queda en la elevacion del blanco, lejos de lo factible).
    if (!n->tilt_activo) {
        for (int i = 0; i <= N; ++i) {
            if (i)
                Xl[i][1] = n->tilt_fijo;
            Xl[i][3] = Xl[i][5] = 0.0;
        }
        for (int i = 0; i < N; ++i)
            Ul[i][1] = 0.0;
    }

    double fov1[M_FOV]; // retranqueos de la etapa 1, para el overlay
    // 2. Parametros por etapa, con los retranqueos de la FOV evaluados en Xl (zoRO).
    for (int i = 0; i <= N; ++i) {
        double p[NPAR], Hq[4], Hp[4], Hr[2];
        p[0] = psi[i][0];
        p[1] = psi[i][1];
        p[2] = psid[i][0];
        p[3] = psid[i][1];
        p[4] = rho;
        geo_rumbo_jac(&n->geo, Xl[i], psi[i], rho, Hq, Hp, Hr);
        // Sigma_r = R_xi Sigma_xi R_xi^T + sigma_q^2 R_q R_q^T con R_xi = [R_psi 0 0 R_rho], los jacobianos del rumbo.
        double Ss[4];
        for (int a = 0; a < 2; ++a)
            for (int c = 0; c < 2; ++c) {
                const int ia[3] = {0, 1, 6};
                const double ha[3] = {Hp[2 * a], Hp[2 * a + 1], Hr[a]}, hc[3] = {Hp[2 * c], Hp[2 * c + 1], Hr[c]};
                double s = n->sigma_q * n->sigma_q * (Hq[2 * a] * Hq[2 * c] + Hq[2 * a + 1] * Hq[2 * c + 1]);
                for (int k = 0; k < 3; ++k)
                    for (int l = 0; l < 3; ++l)
                        s += ha[k] * Sig[i][ia[k] * EKF_N + ia[l]] * hc[l];
                Ss[2 * a + c] = s;
            }
        for (int j = 0; j < M_FOV; ++j) {
            const double ax = cos(2.0 * M_PI * j / M_FOV), ay = sin(2.0 * M_PI * j / M_FOV);
            const double v = ax * ax * Ss[0] + ax * ay * (Ss[1] + Ss[2]) + ay * ay * Ss[3];
            p[5 + j] = fmin(n->beta_fov * sqrt(fmax(v, 0.0)), n->cfg.b_max);
        }
        if (i == 1)
            memcpy(fov1, p + 5, sizeof(fov1));
        p[13] = n->Jp;
        p[14] = n->Jt;
        p[15] = n->b;
        p[16] = n->tau_c;
        p[17] = n->mgl;
        p[18] = n->tau_max[0];
        p[25] = n->tau_max[1];
        p[19] = n->geo.fu;
        p[20] = n->geo.fv;
        p[21] = n->geo.cu;
        p[22] = n->geo.cv;
        p[23] = n->geo.lx;
        p[24] = n->r_fov;
        nmpc_acados_update_params(cap, i, p, NPAR);
    }

    // 3. Pesos y referencias de etapa.
    const double wv = n->cfg.w_p * kappa * n->t_e * n->t_e / (24.0 * n->sigma0 * n->sigma0);
    double W[NY * NY] = {0};
    const double diag[NY] = {n->cfg.W_s, n->cfg.W_s, wv, wv, n->cfg.W_u, n->cfg.W_u, n->cfg.W_delta, n->cfg.W_delta};
    for (int k = 0; k < NY; ++k)
        W[k * NY + k] = diag[k];
    for (int i = 0; i < N; ++i) {
        const double *ua = i ? us[i - 1] : us[0];
        double y[NY] = {n->geo.cu, n->geo.cv, 0.0, 0.0, us[i][0], us[i][1], us[i][0] - ua[0], us[i][1] - ua[1]};
        ocp_nlp_cost_model_set(cfg, dims, in, i, "yref", y);
        ocp_nlp_cost_model_set(cfg, dims, in, i, "W", W);
    }
    // Costo terminal: x*_N y P_N con R_q* = R_q(q*_N, psi_hat_N, rho_hat) (el jacobiano del rumbo).
    double Hq[4], Hp[4], Hr[2], Qq[4], P[16];
    geo_rumbo_jac(&n->geo, qs[N], psi[N], rho, Hq, Hp, Hr);
    for (int a = 0; a < 2; ++a)
        for (int c = 0; c < 2; ++c)
            Qq[2 * a + c] = n->cfg.W_s * (Hq[a] * Hq[c] + Hq[2 + a] * Hq[2 + c]);
    riccati(Ts, Qq, n->cfg.eps_P, n->cfg.W_u, P);
    double ye[4] = {qs[N][0], qs[N][1], recortar(qds[N][0], -2.0, 2.0), recortar(qds[N][1], -2.0, 2.0)};
    ocp_nlp_cost_model_set(cfg, dims, in, N, "yref", ye);
    ocp_nlp_cost_model_set(cfg, dims, in, N, "W", P);

    // 4. Estado inicial fijo e inicializacion.
    ocp_nlp_constraints_model_set(cfg, dims, in, out, 0, "lbx", Xl[0]);
    ocp_nlp_constraints_model_set(cfg, dims, in, out, 0, "ubx", Xl[0]);
    for (int i = 0; i <= N; ++i) {
        ocp_nlp_out_set(cfg, dims, out, in, i, "x", Xl[i]);
        if (i < N)
            ocp_nlp_out_set(cfg, dims, out, in, i, "u", Ul[i]);
    }
    // Guarda: un dato no finito deja la memoria de HPIPM inservible (ni el reinicio la limpia)
    // y el solver puede quedarse iterando. Pasa con un EKF divergido, cuya referencia de mirada
    // g(psi_hat, rho_hat) no existe.
    int entradas_finitas = isfinite(rho);
    for (int k = 0; k < 16; ++k)
        entradas_finitas &= isfinite(P[k]);
    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j < 6; ++j)
            entradas_finitas &= isfinite(Xl[i][j]);
        for (int j = 0; j < 2; ++j)
            entradas_finitas &= isfinite(psi[i][j]) && isfinite(psid[i][j]) && isfinite(qs[i][j]) && isfinite(qds[i][j])
                                && (i == N || (isfinite(us[i][j]) && isfinite(Ul[i][j])));
        for (int k = 0; k < EKF_N * EKF_N; ++k)
            entradas_finitas &= isfinite(Sig[i][k]);
    }
    if (entradas_finitas) { // para el overlay: solo se guardan
        n->t_a = t_a;
        memcpy(n->fov_b, fov1, sizeof(fov1));
        memcpy(n->psi_h, psi, sizeof(psi));
        n->rho_h = rho;
    }
    const int iter = !entradas_finitas ? 0 : (n->hay_solucion ? 1 : (n->cfg.iter_arranque > 1 ? n->cfg.iter_arranque : 1));
    int estado = entradas_finitas ? 0 : -1;
    n->ms = 0.0;
    n->qp_iter = 0;
    n->ms_qp = 0.0;
    // Tras un QP fallido no se sigue iterando: la memoria de HPIPM queda inservible hasta el
    // reinicio de mas abajo (H17), y una iteracion posterior que "resolviera" partiria de basura.
    for (int k = 0; k < iter; ++k) {
        estado = nmpc_acados_solve(cap);
        double t = 0.0, tq = 0.0;
        int it = 0;
        ocp_nlp_get(sol, "time_tot", &t);
        ocp_nlp_get(sol, "time_qp", &tq);
        ocp_nlp_get(sol, "qp_iter", &it);
        n->ms += 1e3 * t;
        n->ms_qp += 1e3 * tq;
        n->qp_iter += it;
        if (estado != 0)
            break;
    }

    // 5. Solucion, o respaldo con la anterior desplazada.
    double Xs[N + 1][6], Us[N][2];
    int finita = 1;
    for (int i = 0; i <= N; ++i) {
        ocp_nlp_out_get(cfg, dims, out, i, "x", Xs[i]);
        if (i < N)
            ocp_nlp_out_get(cfg, dims, out, i, "u", Us[i]);
        for (int j = 0; j < 6; ++j)
            finita &= isfinite(Xs[i][j]);
        if (i < N)
            finita &= isfinite(Us[i][0]) && isfinite(Us[i][1]);
    }
    // Un QP fallido (paso minimo o NaN en HPIPM) deja la memoria del solver inservible: sin
    // reiniciarla, la iteracion siguiente puede quedar girando indefinidamente.
    if (estado != 0 || !finita)
        nmpc_acados_reset(cap, 1, 0, 0, 0);
    *holgura = 0.0;
    if (estado == 0 && finita) {
        memcpy(n->X, Xs, sizeof(Xs));
        memcpy(n->U, Us, sizeof(Us));
        n->t_sol = t_a;
        n->hay_solucion = 1;
        for (int i = 1; i < N; ++i) {
            double su[M_FOV]; // holguras de las 8 restricciones blandas (la FOV)
            ocp_nlp_out_get(cfg, dims, out, i, "su", su);
            for (int j = 0; j < M_FOV; ++j)
                *holgura = fmax(*holgura, su[j]);
        }
    } else if (n->hay_solucion && entradas_finitas) {
        memcpy(n->U, Ul, sizeof(Ul));
        memcpy(n->X, Xl, sizeof(Xl));
        n->t_sol = t_a;
    } else if (entradas_finitas)
        memcpy(n->U, us, sizeof(us));
    else {
        detenerse(n, x0, U); // sin referencia utilizable
        return estado;
    }
    for (int i = 0; i < N; ++i) {
        U[i][0] = n->U[i][0];
        U[i][1] = n->U[i][1];
    }
    return estado;
}
//---------------------------------------------------------------------------
// Rumbo predicho del blanco en cada nodo del horizonte (para dibujarlo en el dashboard).
int nmpc_horizonte_rumbo(const Nmpc *n, double r[][2])
{
    if (!n->hay_solucion)
        return 0;
    for (int i = 0; i <= N; ++i) {
        geo_rumbo(&n->geo, n->X[i], n->psi_h[i], n->rho_h, r[i]);
        r[i][0] -= n->geo.cu; // relativo al centro, como la FOV
        r[i][1] -= n->geo.cv;
    }
    return N + 1;
}
//---------------------------------------------------------------------------
// Adaptadores con la forma de PlanificadorExterno, para registrarlo en el controlador.
int nmpc_planificar(void *ctx, const EKFPieza *ekf, double t_a, const double x0[4], const double u_prev[2],
                    double kappa, double U[][2], int n_max, double *holgura, int *qp_iter, double *ms_qp)
{
    Nmpc *n = ctx;
    double Ul[N][2];
    const int estado = nmpc_resolver(n, ekf, t_a, x0, u_prev, kappa, Ul, holgura);
    *qp_iter = n->qp_iter;
    *ms_qp = n->ms_qp;
    for (int i = 0; i < n_max && i < N; ++i) {
        U[i][0] = Ul[i][0];
        U[i][1] = Ul[i][1];
    }
    return estado;
}
//---------------------------------------------------------------------------
void nmpc_planificador_reiniciar(void *ctx)
{
    nmpc_reiniciar((Nmpc *)ctx);
}

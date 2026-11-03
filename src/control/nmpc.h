// NMPC SQP-RTI del informe (ec. ocp) sobre el solver que genera acados (codegen/nmpc.py).
//
// Por ciclo (informe, "Ejecucion por ciclo"):
//  1. Fuera del solver, con el blanco predicho por el EKF desde t_a: xi_hat_i, Sigma_xi,i, la
//     referencia de mirada q*_i = g(psi_hat_i, rho_hat), q_dot*_i = dg/dpsi psi_dot_hat_i, u*_i = (q_dot*_{i+1} - q_dot*_i)/T_s,
//     los retranqueos de la FOV (zoRO, en la solucion anterior desplazada) y el costo terminal
//     P_N por Riccati con Q_P = blkdiag(R_q*^T W_s R_q*, eps_P I), R_P = W_u. El error de imagen,
//     la FOV y sus retranqueos se miden con el rumbo r (geo_rumbo, sin la division por a_z),
//     que coincide con h cerca del centro y no acopla el tilt al error horizontal.
//  2. Inicializacion: la solucion anterior desplazada los nodos que pasaron desde su t_a (uno a
//     30 fps, dos a 15), con u*_i en las etapas que quedan libres; sin solucion (arranque o
//     reenganche), la referencia recortada y varias iteraciones SQP.
//  3. Una iteracion SQP-RTI desde x_tilde0 = [x_cmd(t_a); u_{-1}].
//  4. Respaldo: si el QP falla o la solucion no es finita, la anterior desplazada.
//
// Solo se compila con acados (-DACADOS_DIR).
#ifndef NMPC_H
#define NMPC_H

#include "ekf_pieza.h"
#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M_FOV 8 // semiplanos del octagono de la FOV (= DC_M_FOV del dashboard)
#define NMPC_HORIZONTE 10 // = nmpc_solver.N de torreta.ini (nmpc.c lo comprueba al compilar)
// nmpc_resolver no planifica porque la prediccion del blanco ya no dice donde esta (P_psi en t_a
// mayor que la FOV): U es un plan de detencion. Igual a PLAN_SIN_BLANCO (ctl_programado.h).
#define NMPC_SIN_BLANCO (-2)

typedef struct {
    ParamsNmpc cfg;
    Geometria geo;
    double Ts, sigma0, t_e, sigma_q, r_fov, beta_fov;
    double sigma_fov2; // (r_fov/f)^2: escala angular de la FOV para el canal de confianza de psi_dot
    double mu_min, mu_max; // rango de la elevacion predicha: el del tilt
    int tilt_activo;       // 0: tilt sin motor, fijo en tilt_fijo (rad) en todo el horizonte
    double tilt_fijo;
    double Jp, Jt, b, tau_c, mgl, tau_max[2]; // tau_max por eje: eta K_m I_p r_j
    void *capsula; // nmpc_solver_capsule
    int hay_solucion;
    double X[NMPC_HORIZONTE + 1][6], U[NMPC_HORIZONTE][2];
    double t_sol;  // t_a de X y U: el ciclo siguiente las desplaza los nodos que pasaron desde ahi
    double ms;     // tiempo del solver en el ultimo ciclo (ms)
    int qp_iter;   // iteraciones de HPIPM en el ultimo ciclo (sumadas sobre las iteraciones SQP)
    double ms_qp;  // de ese tiempo, el del QP (ms)
    // Para el overlay del dashboard, del ultimo ciclo con entradas finitas (se guardan tras esa
    // comprobacion; no entran al solver): t_a, los retranqueos b_j (px de rumbo) de la etapa 1
    // (la primera predicha) y la psi_hat_i, rho_hat con que se armo el solver (la prediccion ya encogida
    // por confianza_a / confianza_v, no la del EKF crudo).
    double t_a, fov_b[M_FOV], psi_h[NMPC_HORIZONTE + 1][2], rho_h;
} Nmpc;

// Devuelve 0 si el solver no se pudo crear.
int nmpc_iniciar(Nmpc *n, const Params *modelo);
void nmpc_liberar(Nmpc *n);

// Olvida la solucion anterior (el proximo ciclo se inicializa con la referencia).
void nmpc_reiniciar(Nmpc *n);

// Resuelve desde x0 = [q; q_dot] en el instante t_a (el del EKF), con la aceleracion del ultimo
// tramo comprometido u_prev y el endurecimiento de percepcion kappa >= 1. Escribe la secuencia
// de aceleraciones U (N x 2). Devuelve el estado del solver (0 = bien) o NMPC_SIN_BLANCO;
// *holgura es la mayor holgura de FOV (px de rumbo) de la solucion.
int nmpc_resolver(Nmpc *n, const EKFPieza *ekf, double t_a, const double x0[4], const double u_prev[2],
                  double kappa, double U[][2], double *holgura);

// Rumbo r_i = geo_rumbo(q_i, psi_hat_i, rho_hat) del blanco predicho en cada etapa del plan (i = 0..N),
// con q_i = X[i] y las psi_hat_i, rho_hat guardadas del solver (psi_h, rho_h): no depende del EKF. r es
// relativo a (cu, cv) y esta en px de rumbo (la medida de la FOV y de sus retranqueos).
// Devuelve N + 1, o 0 si no hay solucion (arranque, reinicio, plan de detencion o ciclo sin
// referencia utilizable).
int nmpc_horizonte_rumbo(const Nmpc *n, double r[][2]);

// Adaptadores con la firma de PlanificadorExterno (ctl_programado.h).
int nmpc_planificar(void *ctx, const EKFPieza *ekf, double t_a, const double x0[4], const double u_prev[2],
                    double kappa, double U[][2], int n_max, double *holgura, int *qp_iter, double *ms_qp);
void nmpc_planificador_reiniciar(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* NMPC_H */

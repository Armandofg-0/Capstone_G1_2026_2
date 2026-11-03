// Controlador del ciclo de la Pi: EKF en la pieza + NMPC con aplicacion programada adaptativa
// (perdida.tex). Corre en la Pi y habla con el nucleo del firmware por el enlace:
//
//  1. Telemetria: sincronizacion de relojes (Cristian, con t1 del plan, tau2 y tau3 de la ESP32 y
//     t4 de llegada), resincronizacion de Phi, historial de la pose q_hat (en el reloj de la Pi) y
//     monitor de perdidas de Gilbert-Elliott (planes que nunca llegaron a estar vigentes).
//  2. EKF: correccion con la pose en el instante de captura t_m (todo en el reloj de la Pi).
//  3. t_a = t_s + Delta_fb, llevado al tick de la ESP32 y a su grilla de ciclos.
//  4. x0 = Phi(t_a): el estado comandado que tendra la ESP32 en t_a ([PROPUESTA]: el MPC parte
//     de x_cmd(t_a)). xi_hat(t_a) por prediccion del EKF.
//  5. NMPC desde x0 y la prediccion del blanco en t_a, sin la compensacion de un
//     periodo (Phi ya recorrio el retardo).
//  6. Plan: nodo 0 = q_dot_cmd(t_a), M nodos con la secuencia optima, N_ff de cola con la velocidad
//     de mirada q_dot*_i = dg/dpsi psi_dot_hat_i del blanco predicho; la ESP32 frena al agotarse la cola.
//     Sin estimacion del blanco (o si el planificador devuelve PLAN_SIN_BLANCO), el plan
//     vuelve a la pose desde la que se lo vio por ultima vez, con un perfil de minimo tiempo
//     dentro de q_dot_max y q_ddot_max, y la cola queda en reposo: quedarse donde se lo perdio, o
//     seguir la deriva del EKF, deja la camara lejos de donde el blanco puede reaparecer.
#ifndef CTL_PROGRAMADO_H
#define CTL_PROGRAMADO_H

#include "ekf_pieza.h"
#include "gilbert_elliott.h"
#include "params.h"
#include "predictor.h"
#include "interfaz_ctl.h"
#include "sincronizacion.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROG_HIST 512
#define PROG_ENVIADOS 64
#define PROG_N_MAX 16 // nodos de aceleracion del planificador
#define PLAN_SIN_BLANCO (-2) // el planificador externo no ve al blanco: buscar

// Planificador (el NMPC, src/control/nmpc.h): recibe el
// EKF, t_a, x0 = x_cmd(t_a), la aceleracion del ultimo tramo comprometido y kappa, y escribe hasta
// n_max aceleraciones. Devuelve 0 si resolvio bien. Diagnostico: la mayor holgura de FOV, y las
// iteraciones y el tiempo (ms) del QP.
// Nivel de actuacion (niveles del dashboard):
//  - CTL_ACTUAR: el lazo completo (el comportamiento de siempre);
//  - CTL_SOMBRA: planifica y codifica el plan, pero no lo registra en Phi ni en el monitor de
//    perdidas: nada se ejecuta, y Phi predice x0 = estado de la telemetria. p_hat_GE y el margen
//    quedan en NaN;
//  - CTL_SOLO_ESTIMAR: telemetria y EKF, sin planificar ni producir plan.
typedef enum { CTL_ACTUAR = 0, CTL_SOMBRA, CTL_SOLO_ESTIMAR } ActuacionCtl;

typedef int (*PlanificadorExterno)(void *ctx, const EKFPieza *ekf, double t_a, const double x0[4],
                                   const double u_prev[2], double kappa, double U[][2], int n_max,
                                   double *holgura, int *qp_iter, double *ms_qp);

typedef struct {
    EKFPieza ekf;
    Geometria geo;
    Sincronizacion sync;
    Predictor phi;
    MonitorGE monitor;
    double gamma;     // Quevedo-Nesic para los pesos del NMPC
    double Ts, delta_fb, qd_max, qdd_max;
    double mu_min, mu_max; // rango del tilt, para la elevacion extrapolada de la cola
    int tilt_activo;       // 0: tilt sin motor, fijo en tilt_fijo (rad) y sin consignas
    double tilt_fijo;
    int M, N_ff, alto;
    int M_pedido; // M de los parametros (se recorta a PROG_N_MAX)
    // Pose q_hat de la telemetria, en el reloj de la Pi.
    long n_hist;
    double h_t[PROG_HIST], h_q[PROG_HIST][2];
    // Planes enviados: secuencia, t_a (tick) y fin de su transmision (reloj de la Pi).
    int n_env;
    uint16_t env_sec[PROG_ENVIADOS];
    uint32_t env_ta[PROG_ENVIADOS];
    double env_t1[PROG_ENVIADOS];
    uint16_t secuencia;
    uint32_t tau2_usado, tau_tel;
    int hay_tel;
    // Monitor de perdidas: el ultimo plan visto vigente.
    int hay_vigente;
    uint16_t ultimo_vigente;
    // Planificador (el NMPC; NULL: sin planificador, solo busca).
    PlanificadorExterno plan;
    void (*plan_reiniciar)(void *ctx);
    void *plan_ctx;
    int ekf_estaba;
    double sigma0, sigma_ultima;
    // Pose de la ultima medicion aceptada (reloj de la Pi): adonde volver a buscar.
    int hay_avistamiento;
    double q_avistamiento[2];
    ActuacionCtl actuacion; // CTL_ACTUAR al iniciar
} CtlProgramado;

void ctl_programado_iniciar(CtlProgramado *c, const Params *modelo);
Controlador ctl_programado_interfaz(CtlProgramado *c);

// Fija el planificador (el NMPC). reiniciar se llama cuando el
// blanco se vuelve a enganchar (sin solucion anterior valida).
void ctl_programado_usar_planificador(CtlProgramado *c, PlanificadorExterno plan, void (*reiniciar)(void *),
                                      void *ctx);

void ctl_programado_fijar_actuacion(CtlProgramado *c, ActuacionCtl a);
// Pose q_hat en el instante t (reloj de la Pi), interpolada en el historial de la telemetria;
// pasada la ultima muestra, la ultima (sin extrapolar). Devuelve 0 si todavia no llego
// ninguna telemetria (q queda sin tocar). La usa la pista de la percepcion en el nivel 4.
int ctl_programado_pose(const CtlProgramado *c, double t, double q[2]);

// Configuracion del nucleo del ejecutor a partir de los parametros (la usan la ESP32 emulada
// y el espejo de Phi).
ConfigEjecutor config_ejecutor(const Params *p);

#ifdef __cplusplus
}
#endif

#endif /* CTL_PROGRAMADO_H */

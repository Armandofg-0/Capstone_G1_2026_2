// Nucleo del ejecutor de la ESP32 (perdida.tex, "El ejecutor de la ESP32"), portable: es el
// emulador del SIL, el espejo que usa la Pi en el predictor Phi y el codigo que se embarca
// (los drivers de ESP-IDF solo le entregan bytes, ticks y el encoder, y toman los pasos).
//
// Cada 1 ms:
//  1. Seleccion: entre los planes validos (CRC correcto) con t_a <= tau, el de mayor secuencia.
//  2. Consigna: interpolacion lineal de q_dot entre los nodos del plan vigente, situados en
//     t_a + j T_s (aceleracion constante por tramo = el modelo del MPC). Terminados los M
//     nodos optimos sigue la cola; terminada la cola, frena con a_frenado y lo informa
//     (enlace perdido: CTL-6, ROB-3).
//  3. Limitador: |Delta q_dot| <= a_lim Delta tau. En operacion nominal el perfil es continuo y no actua;
//     solo actua en los empalmes tras una perdida o un paquete tardio.
//  4. Limites: si con la desaceleracion de frenado el eje no alcanza a detenerse antes del
//     limite de recorrido (menos un margen), frena aunque el plan diga otra cosa (ACT-4,
//     ROB-5). Tope de velocidad qd_max.
//  5. DDA: la posicion comandada del driver es q_cmd + kp e_bar (feedforward mas correccion
//     proporcional del angulo de carga), redondeada al micropaso. e_bar es el error q_cmd - q_enc,
//     filtrado con constante de tiempo tau_correccion (0 = sin filtro). Ojo: la resonancia del
//     stepper (~30 Hz) esta casi sin amortiguar, y la correccion desde el encoder, con su
//     retardo, puede desestabilizarla; en el SIL ninguna combinacion resulto robusta y el
//     valor por defecto es kp = 0 (hallazgo H13 de la fase 5).
//  6. Observador alfa-beta sobre el error e = q_cmd - q_enc: e_hat y q_hat = q_cmd - e_hat van a la telemetria.
//
// Sin memoria dinamica. Los ticks tau son de 32 bits en us (vuelta cada 71 min): las
// diferencias se toman con aritmetica modular.
#ifndef EJECUTOR_H
#define EJECUTOR_H

#include <stdint.h>

#include "mensajes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EJ_PLANES 4 // planes guardados (el vigente y los que esperan su t_a)

typedef struct {
    double Ts;            // separacion de los nodos del plan (s): el periodo del lazo
    double dt;            // periodo del ejecutor (s)
    double a_lim;         // limitador de aceleracion (rad/s^2)
    double a_frenado;     // desaceleracion del frenado y de los limites (rad/s^2)
    double q_min[2], q_max[2], margen_limite; // recorrido permitido (rad)
    double qd_max;        // tope de velocidad (rad/s)
    double micropaso[2];  // rad del eje por micropaso (incluye la relacion de transmision)
    double kp;            // correccion proporcional del DDA
    double tau_correccion; // constante de tiempo del filtro del error de la correccion (s)
    double alfa, beta;    // observador alfa-beta
} ConfigEjecutor;

typedef struct {
    ConfigEjecutor cfg;
    MsgPlan planes[EJ_PLANES];
    int n_planes;
    int hay_vigente;
    MsgPlan vigente;
    uint16_t secuencia;     // secuencia del plan vigente
    uint8_t indice, edad;   // nodo en ejecucion y edad del plan (en periodos T_s)
    double q_cmd[2], qd_cmd[2]; // trayectoria comandada
    long pasos[2];          // posicion comandada del driver, en micropasos
    double e_hat[2], ed_hat[2]; // observador del angulo de carga
    double e_lento[2];      // error filtrado para la correccion del DDA
    uint8_t banderas;       // banderas del ultimo ciclo (TEL_*)
    long n_limitador;       // ciclos con el limitador activo (metrica)
    uint32_t tau2;          // tick en que llego el ultimo plan valido
} NucleoEjecutor;

void nucleo_iniciar(NucleoEjecutor *e, const ConfigEjecutor *cfg, const double q0[2]);

// Decodifica un plan recibido en el tick tau. Devuelve 0 si el CRC o el largo no son validos.
int nucleo_recibir(NucleoEjecutor *e, const uint8_t *msg, int n, uint32_t tau);

// Guarda un plan ya decodificado (lo usa el predictor Phi de la Pi).
void nucleo_agregar_plan(NucleoEjecutor *e, const MsgPlan *p);

// Un ciclo del ejecutor en el tick tau, con la lectura del encoder.
void nucleo_ciclo(NucleoEjecutor *e, uint32_t tau, const double q_enc[2]);

// Telemetria del estado actual (tau3 = tau).
void nucleo_telemetria(const NucleoEjecutor *e, uint32_t tau, MsgTelemetria *t);

// Velocidad que manda el plan p en el tick tau (sin limitador ni limites), y si ya se
// termino la cola (en ese caso, *agotado = 1). La usan el nucleo y el predictor Phi.
double nucleo_consigna_plan(const MsgPlan *p, double Ts, uint32_t tau, int eje, int *indice, int *agotado);

#ifdef __cplusplus
}
#endif

#endif /* EJECUTOR_H */

// Interfaz entre el ciclo de la Pi y el controlador (ctl_programado): lo que recibe en t_s
// (medicion y telemetria del ejecutor) y lo que devuelve (el plan encuadrado y su diagnostico).
#ifndef INTERFAZ_CTL_H
#define INTERFAZ_CTL_H

#include "mensajes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TEL_MAX 128

// Estado del planificador en el ciclo.
typedef enum {
    QP_SIN_RESOLVER = 0, // no se planifico (sin blanco o sin planificador)
    QP_RESUELTO,         // el NMPC resolvio
    QP_MAX_ITER          // el NMPC no convergio; se usa su mejor iterado
} QPEstado;

// Lo que recibe el controlador en t_s.
typedef struct {
    double t_s;                 // instante verdadero (s); el controlador no deberia usarlo
    int k;                      // indice del cuadro
    int hay_z;                  // hubo deteccion
    double z[2], sigma, tamano; // medicion (px), su incertidumbre (px), tamano aparente (px)
    int64_t t_sof_us;           // SensorTimestamp del cuadro (reloj de la Pi, us)
    double t_fila_us, t_e_us;   // para t_m = t_SOF + v t_fila + t_e/2
    double q_tel[2], qd_tel[2]; // telemetria del ejecutor: encoder y velocidad comandada
    // Muestras de telemetria que llegaron desde el ciclo anterior, con su instante de
    // muestreo en la ESP32 (en esta fase, relojes ideales: tiempo verdadero, s).
    int n_tel;
    double tel_t[TEL_MAX], tel_q[TEL_MAX][2], tel_qd[TEL_MAX][2];
    // Ejecutor del firmware (fase 5): lo que ve la Pi de verdad. Reloj de la Pi (s): el
    // instante actual, la telemetria cruda (35 bytes, puede venir corrupta) con su instante
    // de llegada t4, y el fin de la transmision del plan anterior (t1 del intercambio de
    // sincronizacion). Los SensorTimestamp (t_sof_us) tambien estan en el reloj de la Pi.
    double t_s_pi;
    int n_msg;
    uint8_t msg[TEL_MAX][MSG_TELEMETRIA_BYTES];
    double msg_t4[TEL_MAX];
    double t1_plan_anterior;
} EntradaCtl;

typedef enum { PLAN_ACELERACION = 0, PLAN_VELOCIDAD } TipoPlan;

#define PLAN_MAX_NODOS 64

// Lo que devuelve el controlador, mas su diagnostico para el registro.
typedef struct {
    TipoPlan tipo;
    double U[2]; // PLAN_ACELERACION: aceleracion que el ejecutor mantiene desde que llega
    // Diagnostico (NaN o 0 si no aplica).
    double nis;
    int aceptada;
    int estado_qp; // QPEstado (0 = no se resolvio)
    double holgura;
    double ms;     // tiempo de computo medido (ms)
    int qp_iter;   // iteraciones del QP del planificador (0 si no aplica)
    double ms_qp;  // tiempo del QP del planificador (ms)
    // Ejecutor del firmware: el plan ya encuadrado para la UART (n_bytes = 0: no se envia),
    // y la prediccion x_hat(t_a) de Phi para la metrica del error de prediccion.
    int n_bytes;
    uint8_t bytes[2 * MSG_PLAN_MAX_BYTES];
    int hay_prediccion;
    uint32_t tau_a;
    double x_ta[4];
    double ge_p, margen; // monitor de perdidas: p_hat_GE y 1 - p_hat_GE gamma
} SalidaCtl;

typedef struct {
    void *ctx;
    void (*paso)(void *ctx, const EntradaCtl *in, SalidaCtl *out);
    int usa_firmware; // 1: planes de velocidad por el enlace al nucleo del firmware (fase 5)
} Controlador;

#ifdef __cplusplus
}
#endif

#endif /* INTERFAZ_CTL_H */

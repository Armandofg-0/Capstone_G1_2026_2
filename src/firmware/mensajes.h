// Mensajes entre la Pi y la ESP32 (perdida.tex, "Interfaz y costo"). Nucleo portable.
//
// Plan (Pi -> ESP32), 11 + 4 (M + N_ff + 1) bytes:
//   secuencia (2) | t_a en ticks de la ESP32 (4) | M (1) | N_ff (1) | banderas (1) |
//   nodos q_dot por eje, int16 a 1e-4 rad/s (2 x 2 por nodo) | CRC16 (2)
// El nodo 0 es la velocidad en t_a; los nodos 1..M son los del plan optimo en t_a + j T_s,
// y los M+1..M+N_ff, la cola con la velocidad de mirada predicha.
//
// Telemetria (ESP32 -> Pi), 35 bytes:
//   tau (4) | q_hat (2 x float32) | e_hat (2 x float32) | secuencia (2) e indice (1) en ejecucion |
//   edad del buffer (1) | banderas (1) | tau2 (4) | tau3 (4) | CRC16 (2)
// tau2 es el tick en que llego el ultimo plan y tau3 el tick en que sale esta telemetria:
// con el t1 y el t4 de la Pi forman el intercambio de sincronizacion de Cristian.
//
// Calibracion de los ejes, solo con la Pi en los niveles 0 a 3:
//   comando (Pi -> ESP32), 9 bytes: 0xC0 | orden (1) | eje (1) | valor (float32) | CRC16 (2)
//   estado de calibracion (ESP32 -> Pi), 30 bytes, a 10 Hz:
//     0xC1 | banderas (1) | sentido por eje (2 x 1) | encoder en la prueba de sentido (2 x float32, grados) |
//     q_min (2 x float32) | q_max (2 x float32) | CRC16 (2)
// Un plan mide al menos 15 bytes y la telemetria 35: los largos no se confunden.
//
// Todo en little-endian.
#ifndef MENSAJES_H
#define MENSAJES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG_MAX_NODOS 64
#define MSG_PLAN_MAX_BYTES (11 + 4 * MSG_MAX_NODOS)
#define MSG_TELEMETRIA_BYTES 35
#define MSG_RES_VELOCIDAD 1e-4 // rad/s por unidad
#define MSG_COMANDO_BYTES 9
#define MSG_ESTADO_CALIB_BYTES 30

// Ordenes (valor solo se usa en CMD_MOVER, en grados del eje; eje, en las de un eje). CMD_PARAR y CMD_REINICIAR valen
// tambien en el nivel 4; las demas, solo en los niveles 0 a 3.
enum {
    CMD_ENTRAR = 1, // modo calibracion: el nucleo se detiene y se ignoran los planes
    CMD_SALIR,      // vuelve al nucleo (solo si la pose queda dentro de los limites)
    CMD_MOVER,      // mueve el eje `valor` grados
    CMD_CERO,       // cero de ambos ejes en la pose actual
    CMD_MINIMO,     // q_min del eje = pose actual
    CMD_MAXIMO,     // q_max del eje = pose actual
    CMD_GUARDAR,    // cero y limites a la NVS
    CMD_PARAR,      // parada de emergencia: deshabilita los drivers ya (en cualquier nivel)
    CMD_REINICIAR,  // reinicia la ESP32 (rearma los drivers tras una parada o una falla)
    CMD_SENTIDO     // prueba de sentido del eje: +10 grados y vuelta (como `d E` de la consola)
};
// Banderas del estado de calibracion.
#define CALIB_CALIBRANDO 0x01 // en modo calibracion
#define CALIB_GUARDADA 0x02   // hay una calibracion guardada en la NVS
#define CALIB_ENC0 0x04       // el AS5600 del pan respondio al arrancar
#define CALIB_ENC1 0x08       // el del tilt
#define CALIB_INA0 0x10       // el INA219 del pan respondio al arrancar
#define CALIB_INA1 0x20       // el del tilt
// Resultado de la ultima prueba de sentido de un eje.
enum {
    SENTIDO_NINGUNO = 0,  // no se ha probado desde el arranque
    SENTIDO_OK,           // el encoder acompano al comando (mismo signo, al menos la mitad)
    SENTIDO_NO_ACOMPANA,  // el encoder no acompano: iman, signo_enc o el motor no giro
    SENTIDO_SIN_ESPACIO,  // no cabian +10 grados dentro de los limites
    SENTIDO_SIN_ENCODER   // el AS5600 no respondio al arrancar
};

// Banderas del plan.
#define PLAN_MODO_SEGURO 0x01 // la Pi pide frenar y quedarse quieta

// Banderas de la telemetria.
#define TEL_LIMITADOR 0x01   // el limitador de aceleracion actuo en este periodo
#define TEL_FRENANDO 0x02    // se agoto la cola: frenado y enlace perdido (CTL-6, ROB-3)
#define TEL_LIMITE 0x04      // un eje esta frenado por un limite de recorrido (ACT-4)
#define TEL_EN_COLA 0x08     // se esta ejecutando la cola de feedforward
#define TEL_SIN_PLAN 0x10    // sin plan vigente: no llego ninguno, o se agoto la cola y los ejes se detuvieron
// Salud de la placa (las pone el firmware, no el nucleo):
#define TEL_DRIVERS_OFF 0x20   // drivers deshabilitados (sobrecorriente o encoder fuera de limites)
#define TEL_FALLA_ENCODER 0x40 // un AS5600 no respondio desde la telemetria anterior
#define TEL_FALLA_INA 0x80     // un INA219 no respondio desde la telemetria anterior

typedef struct {
    uint16_t secuencia;
    uint32_t t_a;
    uint8_t M, N_ff, banderas;
    int16_t nodos[MSG_MAX_NODOS][2];
} MsgPlan;

typedef struct {
    uint32_t tau;
    float q_hat[2], e_hat[2];
    uint16_t secuencia;
    uint8_t indice, edad, banderas;
    uint32_t tau2, tau3;
} MsgTelemetria;

typedef struct {
    uint8_t orden, eje;
    float valor;
} MsgComando;

typedef struct {
    uint8_t banderas;
    uint8_t sentido[2];  // SENTIDO_*
    float dq_sentido[2]; // lo que midio el encoder en la prueba (grados)
    float q_min[2], q_max[2];
} MsgEstadoCalib;

int16_t msg_velocidad_a_int16(double qd);
double msg_int16_a_velocidad(int16_t v);

// Devuelven la cantidad de bytes escritos (0 si el plan no cabe).
int msg_plan_codificar(const MsgPlan *p, uint8_t *buf);
int msg_telemetria_codificar(const MsgTelemetria *t, uint8_t *buf);

// Devuelven 1 si el largo es el esperado y el CRC es correcto.
int msg_plan_decodificar(const uint8_t *buf, int n, MsgPlan *p);
int msg_telemetria_decodificar(const uint8_t *buf, int n, MsgTelemetria *t);
int msg_comando_codificar(const MsgComando *c, uint8_t *buf);
int msg_comando_decodificar(const uint8_t *buf, int n, MsgComando *c);
int msg_estado_calib_codificar(const MsgEstadoCalib *e, uint8_t *buf);
int msg_estado_calib_decodificar(const uint8_t *buf, int n, MsgEstadoCalib *e);

#ifdef __cplusplus
}
#endif

#endif /* MENSAJES_H */

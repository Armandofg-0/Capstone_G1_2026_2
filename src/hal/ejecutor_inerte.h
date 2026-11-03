// Ejecutor inerte: hace de ESP32 sin mover nada. Recibe las tramas de plan, las valida (trama,
// CRC y largo) y las cuenta, pero no aplica ninguna: responde con una telemetria fija en q0.
// Lo usa el modo vivo del dashboard para ejercitar el enlace Pi-ESP32 sin planta ni hardware.
#ifndef EJECUTOR_INERTE_H
#define EJECUTOR_INERTE_H

#include <stdint.h>

#include "mensajes.h"
#include "trama.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double q0[2];
    LectorTrama lector;
    long planes_validos, planes_invalidos;
    uint16_t ultima_sec;
    int en_trama; // hubo bytes distintos de 0x00 desde el ultimo delimitador
} EjecutorInerte;

void inerte_iniciar(EjecutorInerte *e, const double q0[2]);

// Valida trama, CRC y plan; cuenta; no aplica. Una trama que llega a su delimitador y no
// decodifica (COBS invalido, desborde, CRC o largo malos) suma a planes_invalidos; los
// delimitadores sueltos no cuentan.
void inerte_recibir(EjecutorInerte *e, const uint8_t *bytes, int n);

// Telemetria de 35 bytes con tau = tau_us: q_hat = q0, e_hat = 0, tau2 = tau3 = tau_us, banderas =
// TEL_SIN_PLAN, secuencia = 0 (ningun plan llega a estar vigente). Devuelve MSG_TELEMETRIA_BYTES.
int inerte_telemetria(const EjecutorInerte *e, uint32_t tau_us, uint8_t msg[MSG_TELEMETRIA_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* EJECUTOR_INERTE_H */

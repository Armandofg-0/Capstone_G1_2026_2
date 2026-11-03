// Encuadre de mensajes en el flujo de bytes de la UART: COBS (Consistent Overhead Byte
// Stuffing) con 0x00 como delimitador. COBS elimina los ceros del mensaje con un costo de un
// byte cada 254, asi que un 0x00 marca sin ambiguedad el fin de una trama y el lector se
// resincroniza solo tras bytes perdidos o basura. Nucleo portable.
#ifndef TRAMA_H
#define TRAMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRAMA_MAX 400 // bytes de un mensaje decodificado

// Codifica n bytes y agrega el delimitador. Devuelve el largo escrito (n + 2 como maximo
// para n < 254).
int trama_codificar(const uint8_t *msg, int n, uint8_t *salida);

typedef struct {
    uint8_t buf[TRAMA_MAX + 2];
    int n;
    uint8_t salida[TRAMA_MAX]; // ultimo mensaje decodificado
} LectorTrama;

void trama_iniciar(LectorTrama *l);

// Agrega un byte del flujo. Cuando completa una trama valida, la deja en l->salida y
// devuelve su largo; si no, devuelve 0 (las tramas mal formadas se descartan).
int trama_agregar(LectorTrama *l, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* TRAMA_H */

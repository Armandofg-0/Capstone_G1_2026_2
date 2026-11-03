// CRC-16/CCITT-FALSE (polinomio 0x1021, valor inicial 0xFFFF, sin reflexion ni XOR final).
// Nucleo portable: lo usan la Pi y la ESP32.
#ifndef CRC16_H
#define CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16(const uint8_t *datos, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* CRC16_H */

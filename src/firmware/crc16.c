// CRC-16/CCITT-FALSE (ver crc16.h), bit a bit: sin tabla, para no ocupar RAM en la ESP32
// (con mensajes de ~130 bytes son unos pocos us).
#include "crc16.h"

// El CRC es una "huella" de 16 bits de los datos: si un solo bit cambia en el cable, la huella
// ya no coincide y el mensaje se descarta. 0x1021 es el polinomio estandar CCITT.
uint16_t crc16(const uint8_t *datos, size_t n)
{
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        c ^= (uint16_t)(datos[i] << 8);
        for (int b = 0; b < 8; ++b)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

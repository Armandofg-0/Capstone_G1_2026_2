// Enlace serie real Pi <-> ESP32 (UART, 8N1, sin control de flujo, sin bloqueo). Lo usa el
// modo vivo en el nivel 4; en los niveles 0 a 3 la ESP32 la emula el ejecutor inerte.
// Fuera de POSIX (el PC con MSVC) serie_abrir siempre falla.
#ifndef ENLACE_SERIE_H
#define ENLACE_SERIE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Abre el puerto (p. ej. /dev/serial0) en modo crudo a los baudios dados. Devuelve el
// descriptor, o -1 si falla (puerto inexistente o baudios no soportados).
int serie_abrir(const char *ruta, int baudios);

// Escribe n bytes; devuelve los escritos o -1.
int serie_escribir(int fd, const uint8_t *datos, int n);

// Lee lo que ya llego, hasta cap bytes, sin esperar; devuelve los leidos (0 si no hay) o -1.
int serie_leer(int fd, uint8_t *buf, int cap);

void serie_cerrar(int fd);

#ifdef __cplusplus
}
#endif

#endif /* ENLACE_SERIE_H */

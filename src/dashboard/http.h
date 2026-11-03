// HTTP/1.1 minimo para el dashboard: se analiza solo la linea de peticion y Content-Length;
// el resto de las cabeceras se ignora.
#ifndef HTTP_H
#define HTTP_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HTTP_INCOMPLETA = 0, HTTP_LISTA = 1, HTTP_MAL = -1 } ResultadoHttp;

// largo_cabecera: bytes hasta el final de "\r\n\r\n" inclusive (el cuerpo empieza ahi).
typedef struct { char metodo[8], ruta[64]; size_t largo_cabecera, largo_cuerpo; } Peticion;

// HTTP_LISTA significa cabecera completa; el cuerpo (largo_cuerpo bytes) puede faltar aun.
ResultadoHttp http_analizar(const char *buf, size_t n, Peticion *p);

// Cabecera de respuesta en dst. largo_cuerpo < 0 = stream sin largo (Connection: close).
// Devuelve el largo escrito, o -1 si no cabe.
int http_cabecera(char *dst, size_t cap, int codigo, const char *tipo, long largo_cuerpo);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_H */

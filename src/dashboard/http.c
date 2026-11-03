// Lo minimo de HTTP que necesita el dashboard (ver http.h): entender la peticion que manda el
// navegador y escribir la cabecera de la respuesta. HTTP es texto: una linea como
// "GET /ruta HTTP/1.1", luego cabeceras "Nombre: valor" y una linea en blanco ("\r\n\r\n").
// No se usa ninguna libreria web: el servidor es propio para controlar la latencia.
#include "http.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

// Largo maximo aceptado para el cuerpo; las peticiones del dashboard son de unos pocos bytes.
#define HTTP_CUERPO_MAX 65536

// Busca el fin de las cabeceras (la linea en blanco). Devuelve un puntero a ella, o NULL si
// todavia no llego completa.
static const char *buscar_fin(const char *buf, size_t n) {
    for (size_t i = 0; i + 4 <= n; i++)
        if (buf[i] == '\r' && buf[i + 1] == '\n' && buf[i + 2] == '\r' && buf[i + 3] == '\n')
            return buf + i;
    return NULL;
}

// Compara n caracteres sin distinguir mayusculas de minusculas ("ci" = case insensitive):
// los nombres de cabecera HTTP se pueden escribir de cualquier forma.
static int igual_ci(const char *a, const char *b, size_t n) {
    for (size_t i = 0; i < n; i++)
        if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return 0;
    return 1;
}

// Analiza una peticion recibida: extrae el metodo (GET, POST...), la ruta y el largo del
// cuerpo. Responde INCOMPLETA si faltan bytes por llegar, MAL si el texto no es HTTP valido
// y LISTA si se pudo leer entera. Se revisa cada caracter para que un mensaje raro no
// pueda desbordar los arreglos de la peticion.
ResultadoHttp http_analizar(const char *buf, size_t n, Peticion *p) {
    const char *fin = buscar_fin(buf, n);
    if (!fin) return HTTP_INCOMPLETA;
    const char *cab_fin = fin + 4;
    const char *linea_fin = memchr(buf, '\r', (size_t)(fin - buf) + 1);

    // Linea de peticion: METODO SP /ruta SP HTTP/x.y
    const char *c = buf;
    size_t lm = 0;
    while (c < linea_fin && *c >= 'A' && *c <= 'Z') { c++; lm++; }
    if (lm == 0 || lm >= sizeof p->metodo || c >= linea_fin || *c != ' ') return HTTP_MAL;
    memcpy(p->metodo, buf, lm);
    p->metodo[lm] = '\0';
    c++;
    if (c >= linea_fin || *c != '/') return HTTP_MAL;
    const char *r = c;
    while (c < linea_fin && *c != ' ') {
        if ((unsigned char)*c <= 0x20 || (unsigned char)*c >= 0x7f) return HTTP_MAL;
        c++;
    }
    size_t lr = (size_t)(c - r);
    if (lr >= sizeof p->ruta || c >= linea_fin) return HTTP_MAL;
    memcpy(p->ruta, r, lr);
    p->ruta[lr] = '\0';
    c++;
    if ((size_t)(linea_fin - c) < 5 || memcmp(c, "HTTP/", 5) != 0) return HTTP_MAL;

    // Content-Length entre las cabeceras restantes.
    p->largo_cuerpo = 0;
    static const char clave[] = "content-length:";
    const size_t kl = sizeof clave - 1;
    const char *l = linea_fin + 2;
    while (l < fin + 2) {
        const char *e = memchr(l, '\r', (size_t)(fin + 2 - l));
        if (!e) break;
        if ((size_t)(e - l) > kl && igual_ci(l, clave, kl)) {
            const char *d = l + kl;
            while (d < e && (*d == ' ' || *d == '\t')) d++;
            size_t v = 0;
            int hay = 0;
            while (d < e && *d >= '0' && *d <= '9') {
                v = v * 10 + (size_t)(*d - '0');
                if (v > HTTP_CUERPO_MAX) return HTTP_MAL;
                d++;
                hay = 1;
            }
            if (!hay) return HTTP_MAL;
            p->largo_cuerpo = v;
        }
        l = e + 2;
    }
    p->largo_cabecera = (size_t)(cab_fin - buf);
    return HTTP_LISTA;
}

// Texto que acompana a cada codigo de respuesta HTTP (200 = OK, 404 = no encontrado, etc.).
static const char *texto_codigo(int codigo) {
    switch (codigo) {
    case 200: return "OK";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 409: return "Conflict";
    case 503: return "Service Unavailable";
    default:  return "Error";
    }
}

// Escribe la cabecera de una respuesta. Con largo_cuerpo < 0 no se informa el largo (para
// transmisiones continuas, como el video). "no-store" le pide al navegador no guardar copia.
// Devuelve los bytes escritos, o -1 si no cupo.
int http_cabecera(char *dst, size_t cap, int codigo, const char *tipo, long largo_cuerpo) {
    int n;
    if (largo_cuerpo < 0)
        n = snprintf(dst, cap,
                     "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nCache-Control: no-store\r\n"
                     "Connection: close\r\n\r\n",
                     codigo, texto_codigo(codigo), tipo);
    else
        n = snprintf(dst, cap,
                     "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %ld\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                     codigo, texto_codigo(codigo), tipo, largo_cuerpo);
    if (n < 0 || (size_t)n >= cap) return -1;
    return n;
}

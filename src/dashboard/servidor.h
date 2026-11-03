// Servidor HTTP del dashboard: pagina, video, eventos SSE,
// hora y nivel. Un solo hilo y sin locks: publicar y atender se llaman siempre desde el mismo
// hilo, que no es el del lazo.
//
// Politica central: descartar, nunca encolar. Cada cliente de video o de eventos tiene a lo mas
// una trama "en curso" y una "siguiente". Una trama nueva reemplaza a la que este sin empezar
// (la en curso si aun no salio ningun byte, si no la siguiente) y esa cuenta como descartada;
// una trama a medio enviar nunca se interrumpe. Asi lo pendiente por cliente es a lo mas el
// resto de un cuadro a medio enviar mas un cuadro entero.
//
// clientes_max limita cada tipo de stream por separado (video y eventos): una pestana abre uno
// de cada uno. El que sobra recibe 503.
//
// Rutas: GET / (la pagina), GET /video (stream de tramas TRT1), GET /eventos (SSE), GET /hora,
// POST /nivel (cuerpo: un digito), GET /sistema (JSON fijado por el lazo), POST /ajustes y
// POST /reiniciar (ver servidor_accion_pedida), POST /seleccion, POST /calibracion y POST /camara. Todas las respuestas van con Connection: close.
#ifndef SERVIDOR_H
#define SERVIDOR_H

#include <stddef.h>
#include <stdint.h>

#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Servidor Servidor;

typedef struct {
    int clientes_video, clientes_eventos;  // streams abiertos ahora
    long cuadros_enviados;     // cuadros entregados completos al kernel (suma de clientes)
    long cuadros_descartados;  // cuadros reemplazados sin empezar
    long eventos_descartados;
    long cola_max_bytes;       // maximo, por cliente, de lo que guarda el servidor: lo que falta
                               // de la trama en curso mas la siguiente
    long cola_kernel_max_bytes; // maximo, por cliente, de red_cola_envio; -1 si no esta
                                // disponible (Windows)
} EstadisticasServidor;

// Escucha en p->escucha:p->puerto. NULL, con un mensaje en stderr, si los parametros son
// invalidos (puerto fuera de 1..65535, clientes_max fuera de 1..SERVIDOR_CLIENTES_TOPE, formato
// H.264) o si no pudo escuchar (p. ej. puerto ocupado). La pagina se copia.
#define SERVIDOR_CLIENTES_TOPE 24
Servidor *servidor_crear(const ParamsDashboard *p, const uint8_t *pagina, size_t n_pagina);
void servidor_destruir(Servidor *s);

// Copia meta y jpeg una vez, compartidos por todos los clientes de video. Reemplaza el cuadro
// "siguiente" de cada uno (lo cuenta como descartado si habia uno sin empezar); nunca
// interrumpe un cuadro a medio enviar. No toca los sockets: el envio ocurre en servidor_atender.
void servidor_publicar_cuadro(Servidor *s, const char *meta, uint32_t n_meta, const uint8_t *jpeg,
                              uint32_t n_jpeg);
// Misma politica, para SSE: envia "data: <json>\n\n". Los saltos de linea del JSON (que lo
// cortarian en dos eventos) se reemplazan por espacios.
void servidor_publicar_evento(Servidor *s, const char *json, size_t n);

// Una vuelta: espera hasta espera_ms a que algun socket este listo, acepta, lee peticiones y
// envia lo que quepa sin bloquear. Devuelve 1 si hubo progreso (leyo, envio, acepto o cerro
// algo) y 0 si no: un socket listo sin progreso (p. ej. accept fallando) da 0, para que el
// llamador no gire.
int servidor_atender(Servidor *s, int espera_ms);

// Nivel vigente (lo informan las respuestas de /nivel). bloqueado = 1 rechaza todo POST con 409.
void servidor_fijar_nivel(Servidor *s, int nivel, int bloqueado);
// 1 si hay un pedido de nivel nuevo (lo consume y lo deja en *nivel). Gana el ultimo pedido.
int servidor_nivel_pedido(Servidor *s, int *nivel);
void servidor_estadisticas(const Servidor *s, EstadisticasServidor *e);

// JSON que devuelve GET /sistema (se copia; lo arma el lazo al arrancar).
void servidor_fijar_sistema(Servidor *s, const char *json, size_t n);

// POST /ajustes: lineas seccion.clave=valor, con la clave en PARAMS_CLAVES_AJUSTE y el valor de
// [A-Za-z0-9 _.,+-]. Si todas son validas se escriben a ParamsDashboard.ajustes (temporal +
// rename) y se pide reiniciar el programa. Rangos y opciones los valida la carga al arrancar.

// Acciones que pide la pagina. Nunca se aplican en caliente: el lazo termina el proceso con el
// codigo de salida correspondiente y systemd lo relanza (tools/pi/torreta.service).
// ACCION_NIVEL4 (cuerpo "nivel4") pide reiniciar en el nivel 4 una sola vez: el lazo la acepta
// solo si el chequeo de la ESP32 esta en verde.
// ACCION_APAGAR_PI (cuerpo "apagar") apaga la Pi de forma ordenada (codigo de salida 6).
typedef enum {
    ACCION_NINGUNA = 0, ACCION_REINICIAR_PROGRAMA = 3, ACCION_REINICIAR_PI = 4, ACCION_NIVEL4 = 5,
    ACCION_APAGAR_PI = 6
} AccionServidor;
// La ultima accion pedida (POST /ajustes, POST /reiniciar con cuerpo "programa", "pi" o
// "nivel4" o "apagar"); la consume.
AccionServidor servidor_accion_pedida(Servidor *s);

// POST /seleccion, cuerpo "x y ancho alto" (enteros, px del cuadro, 0..65535): recuadro para el
// perfil general. Devuelve 1 (y lo escribe en r) si hubo un pedido desde la ultima consulta.
int servidor_seleccion_pedida(Servidor *s, int r[4]);

// POST /calibracion, cuerpo "orden eje valor": orden CMD_* (1..10, mensajes.h), eje 0 o 1 y valor en
// grados (|valor| <= 360, solo lo usa CMD_MOVER). Devuelve 1 (y lo escribe) si hubo un pedido.
int servidor_calibracion_pedida(Servidor *s, int *orden, int *eje, float *valor);

// POST /camara, cuerpo "capturar", "calcular" o "borrar": calibracion de la camara (la atiende el
// lazo solo en el nivel 0). Devuelve el pedido (CAMARA_*) y lo consume; CAMARA_NINGUNO si no hay.
enum { CAMARA_NINGUNO = 0, CAMARA_CAPTURAR, CAMARA_CALCULAR, CAMARA_BORRAR };
int servidor_camara_pedida(Servidor *s);

// Cabecera de cada trama de /video: "TRT1", n_meta (u32 LE), n_jpeg (u32 LE). Devuelve 12.
size_t trama_video_cabecera(uint8_t dst[12], uint32_t n_meta, uint32_t n_jpeg);

#ifdef __cplusplus
}
#endif

#endif /* SERVIDOR_H */

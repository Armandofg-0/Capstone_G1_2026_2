// Servidor web del dashboard (ver servidor.h, que explica la politica de "descartar, nunca
// encolar"). Atiende al navegador: le entrega la pagina, el video en vivo, los eventos con los
// datos del lazo y recibe los cambios de nivel. Todo corre en un solo hilo y sin esperas: en
// cada vuelta se revisa que sockets tienen algo que leer o espacio para escribir, y se avanza
// un poco en cada uno.
#include "servidor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "http.h"
#include "red.h"
#include "reloj_pc.h"

#define SNDBUF_CLIENTE (64 * 1024) // cola de envio del kernel por cliente
#define PETICION_MAX 2048          // una peticion sin terminar mas larga se cierra
// Conexiones que aun no son streams (peticion en curso o respuesta unica). Si se agotan, la
// peticion ociosa mas antigua cede su lugar (o, si no hay, la respuesta mas antigua): asi las
// que el navegador abre por adelantado y nunca usa no bloquean a las nuevas.
#define CONEXIONES_EXTRA 8

// Trama compartida entre clientes, con cuenta de referencias: un cuadro se copia una sola vez.
typedef enum { TRAMA_OTRA = 0, TRAMA_CUADRO, TRAMA_EVENTO } ClaseTrama;
typedef struct {
    // Una trama es un bloque de bytes listo para enviar (un cuadro de video, un evento, una
    // respuesta). "refs" cuenta cuantos clientes la estan usando: el mismo cuadro se manda a todos
    // sin copiarlo, y se libera cuando el ultimo termina. "d[]" (arreglo sin tamano al final) es
    // un "miembro flexible" de C: los datos van pegados a la estructura en un solo malloc.
    int refs;
    ClaseTrama clase;
    size_t n;
    uint8_t d[];
} Trama;

typedef enum { CX_LIBRE = 0, CX_PETICION, CX_RESPUESTA, CX_VIDEO, CX_EVENTOS } TipoConexion;

typedef struct {
    Sock s;
    TipoConexion tipo;
    double t_inicio;      // instante de aceptacion (reloj_pc_ms): orden para ceder el lugar
    char pet[PETICION_MAX];
    size_t n_pet;
    Trama *en_curso;      // a medio enviar; nunca se interrumpe
    size_t enviados;      // bytes de en_curso ya entregados al kernel
    Trama *siguiente;     // la proxima; publicar la reemplaza
} Conexion;

struct Servidor {
    Sock escucha;
    int clientes_max;     // streams simultaneos de cada tipo (video, eventos)
    int n_cx;
    Conexion *cx;
    Trama *pagina;        // respuesta completa de GET /, compartida
    int nivel, bloqueado;
    int nivel_pedido, hay_pedido;
    Trama *sistema;       // respuesta de GET /sistema; NULL hasta que el lazo la fija
    char ruta_ajustes[256];
    AccionServidor accion;
    int seleccion[4], hay_seleccion;
    int cal_orden, cal_eje, hay_calibracion;
    int camara;
    float cal_valor;
    EstadisticasServidor est;
    int progreso;         // la vuelta actual de servidor_atender leyo, envio, acepto o cerro
};

// --- Tramas ------------------------------------------------------------------------------

// Reserva una trama de n bytes con una referencia (la de quien la crea).
static Trama *trama_nueva(size_t n, ClaseTrama clase) {
    Trama *t = malloc(sizeof *t + n);
    if (!t) return NULL;
    t->refs = 1;
    t->clase = clase;
    t->n = n;
    return t;
}

// Suelta una referencia; la ultima en soltarse libera la memoria.
static void trama_soltar(Trama *t) {
    if (t && --t->refs == 0) free(t);
}

// Respuesta HTTP completa (cabecera y cuerpo) en una trama.
static Trama *trama_respuesta(int codigo, const char *tipo, const void *cuerpo, size_t n) {
    char cab[256];
    int nc = http_cabecera(cab, sizeof cab, codigo, tipo, (long)n);
    if (nc < 0) return NULL;
    Trama *t = trama_nueva((size_t)nc + n, TRAMA_OTRA);
    if (!t) return NULL;
    memcpy(t->d, cab, (size_t)nc);
    if (n) memcpy(t->d + nc, cuerpo, n);
    return t;
}

// Cabecera de 12 bytes de cada cuadro del stream de video: la marca "TRT1", el largo de los
// metadatos y el largo del JPEG, en 4 bytes cada uno con el byte menos significativo primero.
// Con eso el navegador sabe donde termina cada parte.
size_t trama_video_cabecera(uint8_t dst[12], uint32_t n_meta, uint32_t n_jpeg) {
    memcpy(dst, "TRT1", 4);
    for (int i = 0; i < 4; i++) {
        dst[4 + i] = (uint8_t)(n_meta >> (8 * i));
        dst[8 + i] = (uint8_t)(n_jpeg >> (8 * i));
    }
    return 12;
}

// --- Conexiones --------------------------------------------------------------------------

// Cierra la conexion y suelta sus tramas pendientes; el lugar queda libre para otra.
static void cx_cerrar(Conexion *c) {
    red_cerrar(c->s);
    trama_soltar(c->en_curso);
    trama_soltar(c->siguiente);
    c->s = SOCK_NULO;
    c->tipo = CX_LIBRE;
    c->n_pet = 0;
    c->en_curso = c->siguiente = NULL;
    c->enviados = 0;
}

// Cuantas conexiones abiertas hay de un tipo (por ejemplo, clientes de video).
static int contar(const Servidor *s, TipoConexion tipo) {
    int n = 0;
    for (int i = 0; i < s->n_cx; i++) n += s->cx[i].tipo == tipo;
    return n;
}

// Lo que guarda el servidor para un cliente (lo que falta de en curso mas la siguiente) y,
// aparte, la cola del kernel cuando el sistema la informa.
static void actualizar_cola_max(Servidor *s) {
    for (int i = 0; i < s->n_cx; i++) {
        const Conexion *c = &s->cx[i];
        if (c->tipo == CX_LIBRE) continue;
        long n = 0;
        if (c->en_curso) n += (long)(c->en_curso->n - c->enviados);
        if (c->siguiente) n += (long)c->siguiente->n;
        if (n > s->est.cola_max_bytes) s->est.cola_max_bytes = n;
        long k = red_cola_envio(c->s);
        if (k > s->est.cola_kernel_max_bytes) s->est.cola_kernel_max_bytes = k;
    }
}

// Entrega t a c. Reemplaza la trama sin empezar: la en curso si es de la misma clase y aun no
// salio ningun byte (la cabecera HTTP del stream no se pisa), si no la siguiente. La
// reemplazada se cuenta en *descartados. Con en curso sin empezar, la siguiente esta vacia.
static void encolar(Conexion *c, Trama *t, long *descartados) {
    t->refs++;
    if (!c->en_curso) {
        c->en_curso = t;
        c->enviados = 0;
        return;
    }
    Trama **destino = (c->enviados == 0 && c->en_curso->clase == t->clase) ? &c->en_curso
                                                                            : &c->siguiente;
    if (*destino) {
        trama_soltar(*destino);
        if (descartados) (*descartados)++;
    }
    *destino = t;
}

// Respuesta unica: se envia y despues se cierra la conexion.
static void responder(Conexion *c, Trama *t) {
    if (!t) {
        cx_cerrar(c);
        return;
    }
    c->tipo = CX_RESPUESTA;
    encolar(c, t, NULL);
    trama_soltar(t);
}

// Respuesta corta en JSON con el nivel vigente (la usa POST /nivel).
static void responder_json(Conexion *c, int codigo, int nivel) {
    char cuerpo[32];
    int n = snprintf(cuerpo, sizeof cuerpo, "{\"nivel\":%d}", nivel);
    responder(c, trama_respuesta(codigo, "application/json", cuerpo, (size_t)n));
}

// Convierte la conexion en un "stream": una respuesta que nunca termina, por la que se van
// mandando cuadros (video) o eventos. Si ya hay demasiados clientes de ese tipo, responde
// 503 ("servicio no disponible").
static void abrir_stream(Servidor *s, Conexion *c, TipoConexion tipo, const char *contenido) {
    if (contar(s, tipo) >= s->clientes_max) {
        static const char cuerpo[] = "demasiados clientes\n";
        responder(c, trama_respuesta(503, "text/plain", cuerpo, sizeof cuerpo - 1));
        return;
    }
    char cab[256];
    int nc = http_cabecera(cab, sizeof cab, 200, contenido, -1);
    Trama *t = nc > 0 ? trama_nueva((size_t)nc, TRAMA_OTRA) : NULL;
    if (!t) {
        cx_cerrar(c);
        return;
    }
    memcpy(t->d, cab, (size_t)nc);
    c->tipo = tipo;
    encolar(c, t, NULL);
    trama_soltar(t);
}

// POST /nivel: el navegador pide cambiar el nivel de operacion. Solo se anota el pedido; el
// lazo lo lee con servidor_nivel_pedido y decide. Si el nivel esta bloqueado responde 409.
static void pedir_nivel(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    if (s->bloqueado) {
        responder_json(c, 409, s->nivel);
        return;
    }
    // Un digito 0..3, admitiendo espacios o un fin de linea detras.
    int valido = n >= 1 && cuerpo[0] >= '0' && cuerpo[0] <= '3';
    for (size_t i = 1; valido && i < n; i++)
        valido = cuerpo[i] == ' ' || cuerpo[i] == '\r' || cuerpo[i] == '\n' || cuerpo[i] == '\t';
    if (!valido) {
        responder_json(c, 400, s->nivel);
        return;
    }
    s->nivel_pedido = cuerpo[0] - '0';
    s->hay_pedido = 1;
    responder_json(c, 200, s->nivel_pedido);
}

// 1 si la linea [a, a+n) es una clave de la lista blanca, "=" y un valor de caracteres seguros
// (letras, digitos, espacio y _.,+-); sin comillas ni saltos que puedan romper el .ini.
static int linea_ajuste_valida(const char *a, size_t n) {
    const char *igual = memchr(a, '=', n);
    if (!igual) return 0;
    size_t nk = (size_t)(igual - a);
    int conocida = 0;
    for (int i = 0; PARAMS_CLAVES_AJUSTE[i]; i++)
        conocida |= strlen(PARAMS_CLAVES_AJUSTE[i]) == nk && memcmp(PARAMS_CLAVES_AJUSTE[i], a, nk) == 0;
    if (!conocida || igual + 1 == a + n) return 0;
    for (const char *v = igual + 1; v < a + n; v++)
        if (!((*v >= 'a' && *v <= 'z') || (*v >= 'A' && *v <= 'Z') || (*v >= '0' && *v <= '9') ||
              strchr(" _.,+-", *v)))
            return 0;
    return 1;
}

// POST /ajustes: valida todas las lineas y, solo si todas valen, escribe el archivo entero de
// una vez (temporal + rename: un corte de luz deja el archivo viejo o el nuevo, nunca uno a medias).
static void guardar_ajustes(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    static const char malo[] = "ajuste invalido\n", fallo[] = "no se pudo escribir\n", ok[] = "ok\n";
    for (size_t i = 0; i < n;) {
        size_t j = i;
        while (j < n && cuerpo[j] != '\n') j++;
        size_t fin = j > i && cuerpo[j - 1] == '\r' ? j - 1 : j;
        if (fin > i && !linea_ajuste_valida(cuerpo + i, fin - i)) {
            responder(c, trama_respuesta(400, "text/plain", malo, sizeof malo - 1));
            return;
        }
        i = j + 1;
    }
    char tmp[sizeof s->ruta_ajustes + 8];
    snprintf(tmp, sizeof tmp, "%s.tmp", s->ruta_ajustes);
    FILE *f = fopen(tmp, "wb");
    int bien = f && fwrite(cuerpo, 1, n, f) == n && fflush(f) == 0;
    if (f && fclose(f) != 0) bien = 0;
#ifdef _WIN32
    remove(s->ruta_ajustes);  // en Windows rename no reemplaza; en Linux si, y de forma atomica
#endif
    if (!bien || rename(tmp, s->ruta_ajustes) != 0) {
        responder(c, trama_respuesta(500, "text/plain", fallo, sizeof fallo - 1));
        return;
    }
    s->accion = ACCION_REINICIAR_PROGRAMA;
    responder(c, trama_respuesta(200, "text/plain", ok, sizeof ok - 1));
}

// POST /reiniciar: cuerpo "programa" o "pi".
static void pedir_reinicio(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    static const char malo[] = "programa, pi, apagar o nivel4\n", ok[] = "ok\n";
    while (n && (cuerpo[n - 1] == '\n' || cuerpo[n - 1] == '\r' || cuerpo[n - 1] == ' ')) n--;
    if (n == 8 && memcmp(cuerpo, "programa", 8) == 0) s->accion = ACCION_REINICIAR_PROGRAMA;
    else if (n == 2 && memcmp(cuerpo, "pi", 2) == 0) s->accion = ACCION_REINICIAR_PI;
    else if (n == 6 && memcmp(cuerpo, "nivel4", 6) == 0) s->accion = ACCION_NIVEL4;
    else if (n == 6 && memcmp(cuerpo, "apagar", 6) == 0) s->accion = ACCION_APAGAR_PI;
    else {
        responder(c, trama_respuesta(400, "text/plain", malo, sizeof malo - 1));
        return;
    }
    responder(c, trama_respuesta(200, "text/plain", ok, sizeof ok - 1));
}

// POST /seleccion: cuatro enteros "x y ancho alto" separados por espacios.
static void pedir_seleccion(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    static const char malo[] = "x y ancho alto\n", ok[] = "ok\n";
    char txt[64];
    int r[4], fin = 0;
    const size_t m = n < sizeof txt - 1 ? n : sizeof txt - 1;
    memcpy(txt, cuerpo, m);
    txt[m] = '\0';
    int valido = n < sizeof txt && sscanf(txt, "%d %d %d %d %n", &r[0], &r[1], &r[2], &r[3], &fin) == 4 &&
                 txt[fin] == '\0';
    for (int i = 0; valido && i < 4; i++)
        valido = r[i] >= 0 && r[i] <= 65535;
    if (!valido || r[2] == 0 || r[3] == 0) {
        responder(c, trama_respuesta(400, "text/plain", malo, sizeof malo - 1));
        return;
    }
    memcpy(s->seleccion, r, sizeof r);
    s->hay_seleccion = 1;
    responder(c, trama_respuesta(200, "text/plain", ok, sizeof ok - 1));
}

// POST /calibracion: "orden eje valor" (ver servidor.h).
static void pedir_calibracion(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    static const char malo[] = "orden eje valor\n", ok[] = "ok\n";
    char txt[64];
    int orden, eje, fin = 0;
    float valor;
    const size_t m = n < sizeof txt - 1 ? n : sizeof txt - 1;
    memcpy(txt, cuerpo, m);
    txt[m] = '\0';
    const int valido = n < sizeof txt && sscanf(txt, "%d %d %f %n", &orden, &eje, &valor, &fin) == 3 &&
                       txt[fin] == '\0' && orden >= 1 && orden <= 10 && (eje == 0 || eje == 1) &&
                       valor >= -360.0f && valor <= 360.0f;
    if (!valido) {
        responder(c, trama_respuesta(400, "text/plain", malo, sizeof malo - 1));
        return;
    }
    s->cal_orden = orden;
    s->cal_eje = eje;
    s->cal_valor = valor;
    s->hay_calibracion = 1;
    responder(c, trama_respuesta(200, "text/plain", ok, sizeof ok - 1));
}

// POST /camara: una de tres palabras (ver servidor.h).
static void pedir_camara(Servidor *s, Conexion *c, const char *cuerpo, size_t n) {
    static const char malo[] = "capturar, calcular o borrar\n", ok[] = "ok\n";
    static const char *const palabras[] = {"capturar", "calcular", "borrar"};
    while (n && (cuerpo[n - 1] == '\n' || cuerpo[n - 1] == '\r' || cuerpo[n - 1] == ' ')) n--;
    for (int i = 0; i < 3; i++)
        if (n == strlen(palabras[i]) && memcmp(cuerpo, palabras[i], n) == 0) {
            s->camara = CAMARA_CAPTURAR + i;
            responder(c, trama_respuesta(200, "text/plain", ok, sizeof ok - 1));
            return;
        }
    responder(c, trama_respuesta(400, "text/plain", malo, sizeof malo - 1));
}

// Atiende una peticion completa segun su ruta (ver la lista de rutas en servidor.h). Lo que
// venga despues de "?" en la ruta se ignora (el navegador lo usa para evitar su cache).
static void procesar(Servidor *s, Conexion *c, const Peticion *p) {
    char ruta[sizeof p->ruta];
    strcpy(ruta, p->ruta);
    char *q = strchr(ruta, '?');
    if (q) *q = '\0';
    int get = strcmp(p->metodo, "GET") == 0, post = strcmp(p->metodo, "POST") == 0;

    if (get && strcmp(ruta, "/") == 0) {
        s->pagina->refs++;
        responder(c, s->pagina);
    } else if (get && strcmp(ruta, "/video") == 0) {
        abrir_stream(s, c, CX_VIDEO, "application/octet-stream");
    } else if (get && strcmp(ruta, "/eventos") == 0) {
        abrir_stream(s, c, CX_EVENTOS, "text/event-stream");
    } else if (get && strcmp(ruta, "/hora") == 0) {
        char cuerpo[48];
        int n = snprintf(cuerpo, sizeof cuerpo, "{\"t_ms\":%.3f}", reloj_pc_ms());
        responder(c, trama_respuesta(200, "application/json", cuerpo, (size_t)n));
    } else if (post && strcmp(ruta, "/nivel") == 0) {
        pedir_nivel(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else if (get && strcmp(ruta, "/sistema") == 0 && s->sistema) {
        s->sistema->refs++;
        responder(c, s->sistema);
    } else if (post && strcmp(ruta, "/ajustes") == 0) {
        guardar_ajustes(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else if (post && strcmp(ruta, "/camara") == 0) {
        pedir_camara(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else if (post && strcmp(ruta, "/calibracion") == 0) {
        pedir_calibracion(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else if (post && strcmp(ruta, "/seleccion") == 0) {
        pedir_seleccion(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else if (post && strcmp(ruta, "/reiniciar") == 0) {
        pedir_reinicio(s, c, c->pet + p->largo_cabecera, p->largo_cuerpo);
    } else {
        static const char cuerpo[] = "no encontrado\n";
        responder(c, trama_respuesta(404, "text/plain", cuerpo, sizeof cuerpo - 1));
    }
}

// Lee lo disponible. Mientras la peticion no esta completa la acumula y, al completarse, la
// atiende; despues de eso, lo que llegue se descarta. Devuelve 0 si el cliente cerro, fallo
// o mando una peticion demasiado larga (ya cerrado aqui).
static int leer(Servidor *s, Conexion *c) {
    for (;;) {
        if (c->tipo != CX_PETICION) {
            char basura[512];
            long r = red_recibir(c->s, basura, sizeof basura);
            if (r != 0) s->progreso = 1;
            if (r < 0) {
                cx_cerrar(c);
                return 0;
            }
            if (r == 0) return 1;
            continue;
        }
        if (c->n_pet == PETICION_MAX) {
            cx_cerrar(c);
            return 0;
        }
        long r = red_recibir(c->s, c->pet + c->n_pet, PETICION_MAX - c->n_pet);
        if (r != 0) s->progreso = 1;
        if (r < 0) {
            cx_cerrar(c);
            return 0;
        }
        if (r == 0) return 1;
        c->n_pet += (size_t)r;
        Peticion p;
        ResultadoHttp h = http_analizar(c->pet, c->n_pet, &p);
        if (h == HTTP_MAL) {
            static const char cuerpo[] = "peticion invalida\n";
            responder(c, trama_respuesta(400, "text/plain", cuerpo, sizeof cuerpo - 1));
        } else if (h == HTTP_LISTA && c->n_pet >= p.largo_cabecera + p.largo_cuerpo) {
            procesar(s, c, &p);
        } else if (h == HTTP_LISTA && p.largo_cabecera + p.largo_cuerpo > PETICION_MAX) {
            cx_cerrar(c);
            return 0;
        }
        // Atendida la peticion, lo que siga (incluido un cierre) se ve en la proxima vuelta:
        // asi la respuesta alcanza a salir si el cliente solo cerro su lado de escritura.
        if (c->tipo != CX_PETICION) return 1;
    }
}

// Envia lo que quepa sin bloquear. Al terminar la trama en curso pasa a la siguiente; una
// respuesta unica ya enviada se cierra.
static void enviar(Servidor *s, Conexion *c) {
    while (c->en_curso) {
        Trama *t = c->en_curso;
        long r = red_enviar(c->s, t->d + c->enviados, t->n - c->enviados);
        if (r != 0) s->progreso = 1;
        if (r < 0) {
            cx_cerrar(c);
            return;
        }
        if (r == 0) return;
        c->enviados += (size_t)r;
        if (c->enviados < t->n) continue;
        if (t->clase == TRAMA_CUADRO) s->est.cuadros_enviados++;
        trama_soltar(t);
        c->en_curso = c->siguiente;
        c->siguiente = NULL;
        c->enviados = 0;
    }
    if (c->tipo == CX_RESPUESTA) cx_cerrar(c);
}

// La conexion mas antigua del tipo dado, o -1.
static int mas_antigua(const Servidor *s, TipoConexion tipo) {
    int viejo = -1;
    for (int i = 0; i < s->n_cx; i++)
        if (s->cx[i].tipo == tipo && (viejo < 0 || s->cx[i].t_inicio < s->cx[viejo].t_inicio))
            viejo = i;
    return viejo;
}

// Lugar para una conexion ya aceptada: uno libre o, si no hay, el de la peticion ociosa mas
// antigua y, en ultimo caso, el de la respuesta en curso mas antigua, que se cierran. -1 si
// todos son streams (no ocurre: hay CONEXIONES_EXTRA lugares por sobre los streams).
static int lugar_libre(Servidor *s) {
    for (int i = 0; i < s->n_cx; i++)
        if (s->cx[i].tipo == CX_LIBRE) return i;
    int i = mas_antigua(s, CX_PETICION);
    if (i < 0) i = mas_antigua(s, CX_RESPUESTA);
    if (i >= 0) cx_cerrar(&s->cx[i]);
    return i;
}

// Acepta las conexiones nuevas que esten esperando y les asigna un lugar.
static void aceptar(Servidor *s) {
    for (;;) {
        Sock n = red_aceptar(s->escucha);
        if (n == SOCK_NULO) return;
        s->progreso = 1;
        int i = lugar_libre(s);
        if (i < 0) {
            red_cerrar(n);
            return;
        }
        Conexion *c = &s->cx[i];
        red_fijar_sndbuf(n, SNDBUF_CLIENTE);
        c->s = n;
        c->tipo = CX_PETICION;
        c->t_inicio = reloj_pc_ms();
        c->n_pet = 0;
        // La peticion suele llegar junto con la conexion: se atiende en esta misma vuelta.
        if (leer(s, c) && c->en_curso) enviar(s, c);
    }
}

// --- Interfaz ----------------------------------------------------------------------------

// Crea el servidor: abre el puerto de escucha y prepara la pagina (que ya viene embebida en
// el programa como un arreglo de bytes). Devuelve NULL si no pudo abrir el puerto.
Servidor *servidor_crear(const ParamsDashboard *p, const uint8_t *pagina, size_t n_pagina) {
    if (p->puerto < 1 || p->puerto > 65535) {
        fprintf(stderr, "dashboard: puerto %d fuera de 1..65535\n", p->puerto);
        return NULL;
    }
    if (p->clientes_max < 1 || p->clientes_max > SERVIDOR_CLIENTES_TOPE) {
        fprintf(stderr, "dashboard: clientes_max %d fuera de 1..%d\n", p->clientes_max,
                SERVIDOR_CLIENTES_TOPE);
        return NULL;
    }
    if (p->formato != VIDEO_MJPEG) {
        fprintf(stderr, "dashboard: formato de video no implementado (solo mjpeg)\n");
        return NULL;
    }
    if (!red_iniciar()) {
        fprintf(stderr, "dashboard: no se pudo iniciar la red\n");
        return NULL;
    }
    Servidor *s = calloc(1, sizeof *s);
    if (s) {
        s->n_cx = 2 * p->clientes_max + CONEXIONES_EXTRA;
        s->cx = calloc((size_t)s->n_cx, sizeof *s->cx);
        s->pagina = trama_respuesta(200, "text/html; charset=utf-8", pagina, n_pagina);
        snprintf(s->ruta_ajustes, sizeof s->ruta_ajustes, "%s", p->ajustes);
        s->escucha = red_escuchar(p->escucha, p->puerto);
    }
    if (!s || !s->cx || !s->pagina || s->escucha == SOCK_NULO) {
        if (s && s->escucha == SOCK_NULO)
            fprintf(stderr, "dashboard: no se pudo escuchar en %s:%d (puerto ocupado?)\n",
                    p->escucha, p->puerto);
        if (s) {
            free(s->cx);
            trama_soltar(s->pagina);
            red_cerrar(s->escucha);
            free(s);
        }
        red_terminar();
        return NULL;
    }
    for (int i = 0; i < s->n_cx; i++) s->cx[i].s = SOCK_NULO;
    s->clientes_max = p->clientes_max;
    s->est.cola_kernel_max_bytes = -1;
    return s;
}

// Cierra todas las conexiones y libera el servidor.
void servidor_destruir(Servidor *s) {
    if (!s) return;
    for (int i = 0; i < s->n_cx; i++)
        if (s->cx[i].tipo != CX_LIBRE) cx_cerrar(&s->cx[i]);
    red_cerrar(s->escucha);
    trama_soltar(s->pagina);
    if (s->sistema) trama_soltar(s->sistema);
    free(s->cx);
    free(s);
    red_terminar();
}

// Publica un cuadro de video (metadatos + JPEG) a todos los clientes de video. Se arma una
// sola trama y cada cliente recibe una referencia a ella.
void servidor_publicar_cuadro(Servidor *s, const char *meta, uint32_t n_meta, const uint8_t *jpeg,
                              uint32_t n_jpeg) {
    if (contar(s, CX_VIDEO) == 0) return;
    Trama *t = trama_nueva(12 + (size_t)n_meta + n_jpeg, TRAMA_CUADRO);
    if (!t) return;
    trama_video_cabecera(t->d, n_meta, n_jpeg);
    memcpy(t->d + 12, meta, n_meta);
    memcpy(t->d + 12 + n_meta, jpeg, n_jpeg);
    for (int i = 0; i < s->n_cx; i++)
        if (s->cx[i].tipo == CX_VIDEO) encolar(&s->cx[i], t, &s->est.cuadros_descartados);
    trama_soltar(t);
    actualizar_cola_max(s);
}

// Publica un evento (texto JSON) a todos los clientes de eventos, en el formato SSE
// ("data: ...\n\n") que el navegador lee con EventSource.
void servidor_publicar_evento(Servidor *s, const char *json, size_t n) {
    if (contar(s, CX_EVENTOS) == 0) return;
    static const char ini[] = "data: ", fin[] = "\n\n";
    const size_t ni = sizeof ini - 1, nf = sizeof fin - 1;
    Trama *t = trama_nueva(ni + n + nf, TRAMA_EVENTO);
    if (!t) return;
    memcpy(t->d, ini, ni);
    for (size_t i = 0; i < n; i++)
        t->d[ni + i] = (json[i] == '\n' || json[i] == '\r') ? ' ' : (uint8_t)json[i];
    memcpy(t->d + ni + n, fin, nf);
    for (int i = 0; i < s->n_cx; i++)
        if (s->cx[i].tipo == CX_EVENTOS) encolar(&s->cx[i], t, &s->est.eventos_descartados);
    trama_soltar(t);
    actualizar_cola_max(s);
}

// Una vuelta del servidor: espera como maximo espera_ms a que algun socket este listo y
// avanza en cada uno (leer peticiones, enviar lo pendiente, aceptar conexiones nuevas).
// Devuelve 1 si hubo progreso.
int servidor_atender(Servidor *s, int espera_ms) {
    SockEspera v[RED_ESPERA_MAX];
    int idx[RED_ESPERA_MAX];
    int nv = 0;
    v[nv] = (SockEspera){ s->escucha, 1, 0, 0, 0, 0 };
    idx[nv++] = -1;
    for (int i = 0; i < s->n_cx; i++) {
        Conexion *c = &s->cx[i];
        if (c->tipo == CX_LIBRE) continue;
        // Escribir solo si hay algo que enviar; leer siempre, para notar el cierre.
        v[nv] = (SockEspera){ c->s, 1, c->en_curso != NULL, 0, 0, 0 };
        idx[nv++] = i;
    }
    s->progreso = 0;
    if (red_esperar(v, nv, espera_ms) <= 0) {
        actualizar_cola_max(s);
        return 0;
    }
    // Primero los clientes ya conocidos (aceptar puede reciclar un lugar de la lista).
    for (int k = 1; k < nv; k++) {
        Conexion *c = &s->cx[idx[k]];
        int era_peticion = c->tipo == CX_PETICION;
        if (v[k].puede_leer || v[k].error) {
            if (!leer(s, c)) continue;
            if (v[k].error) {
                cx_cerrar(c);
                s->progreso = 1;
                continue;
            }
        }
        // Una peticion recien atendida intenta enviar su respuesta en esta misma vuelta.
        if (c->en_curso && (v[k].puede_escribir || era_peticion)) enviar(s, c);
    }
    if (v[0].puede_leer) aceptar(s);
    actualizar_cola_max(s);
    return s->progreso;
}

// El lazo informa el nivel vigente y si esta bloqueado, para responder a la pagina.
void servidor_fijar_nivel(Servidor *s, int nivel, int bloqueado) {
    s->nivel = nivel;
    s->bloqueado = bloqueado;
}

// Devuelve 1 (y el nivel) si la pagina pidio un cambio desde la ultima consulta.
int servidor_nivel_pedido(Servidor *s, int *nivel) {
    if (!s->hay_pedido) return 0;
    s->hay_pedido = 0;
    *nivel = s->nivel_pedido;
    return 1;
}

// Contadores del servidor (clientes, cuadros descartados, cola maxima), para el resumen.
void servidor_estadisticas(const Servidor *s, EstadisticasServidor *e) {
    *e = s->est;
    e->clientes_video = contar(s, CX_VIDEO);
    e->clientes_eventos = contar(s, CX_EVENTOS);
}

// El lazo fija el JSON de GET /sistema (una vez, al arrancar).
void servidor_fijar_sistema(Servidor *s, const char *json, size_t n) {
    Trama *t = trama_respuesta(200, "application/json", json, n);
    if (!t) return;
    if (s->sistema) trama_soltar(s->sistema);
    s->sistema = t;
}

// Devuelve la ultima accion pedida por la pagina y la consume.
AccionServidor servidor_accion_pedida(Servidor *s) {
    AccionServidor a = s->accion;
    s->accion = ACCION_NINGUNA;
    return a;
}

// Devuelve el ultimo recuadro pedido por la pagina y lo consume.
int servidor_seleccion_pedida(Servidor *s, int r[4]) {
    if (!s->hay_seleccion) return 0;
    s->hay_seleccion = 0;
    memcpy(r, s->seleccion, sizeof s->seleccion);
    return 1;
}

// Devuelve el ultimo comando de calibracion pedido por la pagina y lo consume.
int servidor_calibracion_pedida(Servidor *s, int *orden, int *eje, float *valor) {
    if (!s->hay_calibracion) return 0;
    s->hay_calibracion = 0;
    *orden = s->cal_orden;
    *eje = s->cal_eje;
    *valor = s->cal_valor;
    return 1;
}

// Devuelve el ultimo pedido de calibracion de la camara y lo consume.
int servidor_camara_pedida(Servidor *s) {
    const int p = s->camara;
    s->camara = CAMARA_NINGUNO;
    return p;
}

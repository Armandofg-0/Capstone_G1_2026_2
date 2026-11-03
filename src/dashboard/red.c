// Red (ver red.h): envoltorio delgado sobre los "sockets", la forma estandar en que un programa
// manda y recibe datos por la red. Windows y Linux tienen casi la misma API con nombres y
// detalles distintos; este archivo los esconde para que el servidor del dashboard sea uno solo.
// Todos los sockets son "no bloqueantes": enviar o recibir nunca deja esperando al programa;
// si no se puede en ese momento, devuelven 0 y se reintenta despues. Asi un navegador lento no
// frena el video de los demas.
#include "red.h"

#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mmsystem.h>
typedef SOCKET sock_nativo;
#define NATIVO_NULO INVALID_SOCKET
#define ULTIMO_ERROR() WSAGetLastError()
#define ES_BLOQUEO(e) ((e) == WSAEWOULDBLOCK)
#define ES_SIN_MEMORIA(e) ((e) == WSAENOBUFS)
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/sockios.h>
#endif
typedef int sock_nativo;
#define NATIVO_NULO (-1)
#define ULTIMO_ERROR() errno
#define ES_BLOQUEO(e) ((e) == EAGAIN || (e) == EWOULDBLOCK || (e) == EINTR)
#define ES_SIN_MEMORIA(e) ((e) == ENOBUFS || (e) == ENOMEM)
#endif

// Conversion entre el tipo propio Sock y el tipo nativo del sistema (en Windows es un entero
// sin signo; en Linux, un int). Las macros de arriba (NATIVO_NULO, ULTIMO_ERROR, ES_BLOQUEO)
// cumplen el mismo papel para los errores.
static Sock a_sock(sock_nativo s) { return s == NATIVO_NULO ? SOCK_NULO : (Sock)s; }
static sock_nativo a_nativo(Sock s) { return s == SOCK_NULO ? NATIVO_NULO : (sock_nativo)s; }

// Cierra un socket con la funcion de cada sistema (closesocket en Windows, close en Linux).
static void cerrar_nativo(sock_nativo s) {
#ifdef _WIN32
    closesocket(s);
#else
    close(s);
#endif
}

// Pone el socket en modo no bloqueante (ver la explicacion al inicio del archivo).
static int hacer_no_bloqueante(sock_nativo s) {
#ifdef _WIN32
    u_long uno = 1;
    return ioctlsocket(s, FIONBIO, &uno) == 0;
#else
    int f = fcntl(s, F_GETFL, 0);
    return f >= 0 && fcntl(s, F_SETFL, f | O_NONBLOCK) == 0;
#endif
}

// Cambia una opcion del socket (setsockopt): tamano del buffer, reutilizar la direccion, etc.
static void fijar_opcion(sock_nativo s, int nivel, int opcion, int valor) {
#ifdef _WIN32
    setsockopt(s, nivel, opcion, (const char *)&valor, sizeof valor);
#else
    setsockopt(s, nivel, opcion, &valor, sizeof valor);
#endif
}

// Arma la direccion IPv4 (ip y puerto) en el formato que piden las funciones de sockets.
static int dir_ipv4(const char *ip, int puerto, struct sockaddr_in *a) {
    memset(a, 0, sizeof *a);
    a->sin_family = AF_INET;
    a->sin_port = htons((unsigned short)puerto);
    return inet_pton(AF_INET, ip, &a->sin_addr) == 1;
}

// En Windows tambien se sube a 1 ms la resolucion del temporizador del proceso: con la de
// fabrica (15,6 ms), un WSAPoll o un Sleep de 2 ms duran ~15,6 ms. WSAStartup/WSACleanup y
// timeBeginPeriod/timeEndPeriod llevan su propio conteo de referencias, asi que cada
// red_iniciar exitoso se empareja con un red_terminar.
int red_iniciar(void) {
#ifdef _WIN32
    WSADATA w;
    if (WSAStartup(MAKEWORD(2, 2), &w) != 0) return 0;
    timeBeginPeriod(1);
    return 1;
#else
    return 1;
#endif
}

// Libera lo que tomo red_iniciar.
void red_terminar(void) {
#ifdef _WIN32
    timeEndPeriod(1);
    WSACleanup();
#endif
}

// Abre un socket que "escucha" en ip:puerto, esperando que se conecten navegadores
// (bind = tomar la direccion, listen = empezar a escuchar, con hasta 8 conexiones en espera).
// SO_REUSEADDR permite reabrir el puerto apenas se cierra el programa, sin esperar el minuto
// que Linux reserva por defecto.
Sock red_escuchar(const char *ip, int puerto) {
    struct sockaddr_in a;
    if (!dir_ipv4(ip, puerto, &a)) return SOCK_NULO;
    sock_nativo s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == NATIVO_NULO) return SOCK_NULO;
#ifdef _WIN32
    fijar_opcion(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, 1);
#else
    fijar_opcion(s, SOL_SOCKET, SO_REUSEADDR, 1);
#endif
    if (bind(s, (struct sockaddr *)&a, sizeof a) != 0 || listen(s, 8) != 0 ||
        !hacer_no_bloqueante(s)) {
        cerrar_nativo(s);
        return SOCK_NULO;
    }
    return a_sock(s);
}

// Acepta una conexion pendiente, si la hay (no espera). Devuelve SOCK_NULO si no habia.
Sock red_aceptar(Sock escucha) {
    sock_nativo s = accept(a_nativo(escucha), NULL, NULL);
    if (s == NATIVO_NULO) return SOCK_NULO;
    // El socket aceptado puede heredar el modo de la escucha o no, segun la plataforma.
    if (!hacer_no_bloqueante(s)) {
        cerrar_nativo(s);
        return SOCK_NULO;
    }
    fijar_opcion(s, IPPROTO_TCP, TCP_NODELAY, 1);
    return a_sock(s);
}

// Conecta como cliente a ip:puerto (lo usan las pruebas, que hacen de navegador).
Sock red_conectar(const char *ip, int puerto) {
    struct sockaddr_in a;
    if (!dir_ipv4(ip, puerto, &a)) return SOCK_NULO;
    sock_nativo s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == NATIVO_NULO) return SOCK_NULO;
    if (connect(s, (struct sockaddr *)&a, sizeof a) != 0 || !hacer_no_bloqueante(s)) {
        cerrar_nativo(s);
        return SOCK_NULO;
    }
    return a_sock(s);
}

// Envia hasta n bytes sin esperar. Devuelve cuantos salieron, 0 si el buffer del sistema esta
// lleno (reintentar luego) o -1 si la conexion se cayo. MSG_NOSIGNAL evita que Linux mate al
// programa con la senal SIGPIPE cuando el otro lado cerro la conexion.
long red_enviar(Sock s, const void *d, size_t n) {
    if (n == 0) return 0;
    if (n > 0x40000000u) n = 0x40000000u;
#ifdef _WIN32
    int r = send(a_nativo(s), (const char *)d, (int)n, 0);
    if (r == SOCKET_ERROR) {
        int e = ULTIMO_ERROR();
        return ES_BLOQUEO(e) || ES_SIN_MEMORIA(e) ? 0 : -1;
    }
    return r;
#else
    int flags = MSG_DONTWAIT;
#ifdef MSG_NOSIGNAL
    flags |= MSG_NOSIGNAL;
#endif
    ssize_t r = send(a_nativo(s), d, n, flags);
    if (r < 0) {
        int e = ULTIMO_ERROR();
        return ES_BLOQUEO(e) || ES_SIN_MEMORIA(e) ? 0 : -1;
    }
    return (long)r;
#endif
}

// Recibe hasta n bytes sin esperar. Devuelve cuantos llegaron, 0 si no habia nada o -1 si la
// conexion se cerro o fallo.
long red_recibir(Sock s, void *d, size_t n) {
    if (n == 0) return 0;
    if (n > 0x40000000u) n = 0x40000000u;
#ifdef _WIN32
    int r = recv(a_nativo(s), (char *)d, (int)n, 0);
    if (r == SOCKET_ERROR) return ES_BLOQUEO(ULTIMO_ERROR()) ? 0 : -1;
    return r == 0 ? -1 : r;
#else
    ssize_t r = recv(a_nativo(s), d, n, MSG_DONTWAIT);
    if (r < 0) return ES_BLOQUEO(ULTIMO_ERROR()) ? 0 : -1;
    return r == 0 ? -1 : (long)r;
#endif
}

// Tamano del buffer de envio del sistema. Uno chico evita que se acumulen cuadros viejos de
// video en camino a un navegador lento (mas latencia).
void red_fijar_sndbuf(Sock s, int bytes) { fijar_opcion(a_nativo(s), SOL_SOCKET, SO_SNDBUF, bytes); }

// Bytes que todavia esperan salir por el socket (solo Linux, con ioctl SIOCOUTQ); -1 si no se
// puede saber. Sirve para decidir si conviene mandar otro cuadro o saltarlo.
long red_cola_envio(Sock s) {
#if defined(__linux__) && defined(SIOCOUTQ)
    int n = 0;
    if (ioctl(a_nativo(s), SIOCOUTQ, &n) != 0) return -1;
    return n;
#else
    (void)s;
    return -1;
#endif
}

// Espera, como maximo espera_ms, a que alguno de los sockets pueda leer o escribir (poll):
// asi el hilo del dashboard duerme en vez de preguntar en un ciclo. Devuelve cuantos quedaron
// listos, 0 si se acabo el tiempo o -1 si hubo un error.
int red_esperar(SockEspera *v, int n, int espera_ms) {
    if (n <= 0) return 0;
    if (n > RED_ESPERA_MAX) n = RED_ESPERA_MAX;
#ifdef _WIN32
    WSAPOLLFD f[RED_ESPERA_MAX];
#else
    struct pollfd f[RED_ESPERA_MAX];
#endif
    for (int i = 0; i < n; i++) {
        f[i].fd = a_nativo(v[i].s);
        f[i].events = (short)((v[i].leer ? POLLIN : 0) | (v[i].escribir ? POLLOUT : 0));
        f[i].revents = 0;
        v[i].puede_leer = v[i].puede_escribir = v[i].error = 0;
    }
#ifdef _WIN32
    int r = WSAPoll(f, (ULONG)n, espera_ms);
    if (r == SOCKET_ERROR) return -1;
#else
    int r = poll(f, (nfds_t)n, espera_ms);
    if (r < 0) return errno == EINTR ? 0 : -1;
#endif
    if (r == 0) return 0;
    int listos = 0;
    for (int i = 0; i < n; i++) {
        v[i].puede_leer = (f[i].revents & (POLLIN | POLLHUP)) != 0;
        v[i].puede_escribir = (f[i].revents & POLLOUT) != 0;
        v[i].error = (f[i].revents & (POLLERR | POLLNVAL)) != 0;
        if (v[i].puede_leer || v[i].puede_escribir || v[i].error) listos++;
    }
    return listos;
}

// Cierra el socket (si es valido).
void red_cerrar(Sock s) {
    if (s != SOCK_NULO) cerrar_nativo(a_nativo(s));
}

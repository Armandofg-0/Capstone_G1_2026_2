// Capa minima de sockets TCP portable (Winsock2 en Windows, POSIX en Linux) para el servidor
// del dashboard. Todo es no bloqueante salvo el connect de red_conectar, que existe solo para
// las pruebas. Nada aqui se llama desde el hilo del lazo.
#ifndef RED_H
#define RED_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// En Windows un SOCKET es UINT_PTR; se guarda en un entero con signo y INVALID_SOCKET se
// convierte a SOCK_NULO en la frontera.
typedef intptr_t Sock;
#define SOCK_NULO ((Sock)-1)

// Windows: WSAStartup y timeBeginPeriod(1) (temporizador a 1 ms); 1 = bien. Cada llamada
// exitosa se empareja con un red_terminar.
int  red_iniciar(void);
void red_terminar(void);

// Escucha no bloqueante. Linux: SO_REUSEADDR; Windows: SO_EXCLUSIVEADDRUSE (con SO_REUSEADDR,
// Winsock permitiria dos escuchas en el mismo puerto). SOCK_NULO si falla (p. ej. puerto ocupado).
Sock red_escuchar(const char *ip, int puerto);
// No bloqueante; SOCK_NULO si no hay nadie. El socket aceptado queda no bloqueante y con
// TCP_NODELAY (sin Nagle, que con el ACK retardado atrasaria ~40 ms los envios chicos).
Sock red_aceptar(Sock escucha);
// Solo para pruebas: el connect bloquea; el socket devuelto queda no bloqueante.
Sock red_conectar(const char *ip, int puerto);
// >0 enviados, 0 = bloquearia (tambien sin memoria de red transitoria: ENOBUFS, ENOMEM,
// WSAENOBUFS), -1 = cerrado/error. Nunca levanta SIGPIPE.
long red_enviar(Sock s, const void *d, size_t n);
long red_recibir(Sock s, void *d, size_t n);         // >0 leidos, 0 = no hay, -1 = cerrado/error
void red_fijar_sndbuf(Sock s, int bytes);
// Bytes en la cola de envio del kernel que el otro extremo aun no confirma (SIOCOUTQ en Linux:
// incluye los ya transmitidos sin ACK, no solo los no enviados). -1 si no esta disponible
// (Windows).
long red_cola_envio(Sock s);

// Espera con poll / WSAPoll a lo mas espera_ms (0 = no esperar, <0 = sin plazo). Contrato:
//   - devuelve cuantos sockets tienen algo que informar, 0 si vencio el plazo o si una senal
//     interrumpio la espera (EINTR: vuelve antes, sin nada listo), -1 si poll fallo;
//   - puede_leer: hay datos o el otro extremo cerro (POLLIN o POLLHUP). Al leer salen primero
//     los datos pendientes y despues -1, asi que el llamador lee antes de soltar al cliente;
//   - puede_escribir: cabe algo en la cola de envio (POLLOUT);
//   - error: el socket fallo (POLLERR o POLLNVAL); leer o escribir devolvera -1.
// A lo mas RED_ESPERA_MAX sockets por llamada.
#define RED_ESPERA_MAX 64
typedef struct { Sock s; int leer, escribir; int puede_leer, puede_escribir, error; } SockEspera;
int  red_esperar(SockEspera *v, int n, int espera_ms);
void red_cerrar(Sock s);

#ifdef __cplusplus
}
#endif

#endif /* RED_H */

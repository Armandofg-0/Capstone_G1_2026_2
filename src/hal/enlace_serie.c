// Enlace serie real (ver enlace_serie.h). Usa termios, la API de Linux para configurar puertos
// serie (velocidad, bits, paridad).
#include "enlace_serie.h"

// En Windows (el PC de desarrollo) no hay UART hacia la ESP32: todas las funciones fallan y
// el modo vivo rechaza el nivel 4. "(void)x" solo evita el aviso de parametro sin usar.
#ifdef _WIN32

int serie_abrir(const char *ruta, int baudios) { (void)ruta; (void)baudios; return -1; }
int serie_escribir(int fd, const uint8_t *datos, int n) { (void)fd; (void)datos; (void)n; return -1; }
int serie_leer(int fd, uint8_t *buf, int cap) { (void)fd; (void)buf; (void)cap; return -1; }
void serie_cerrar(int fd) { (void)fd; }

#else

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// termios no recibe baudios como numero sino como constantes (B921600, etc.): se traduce.
// Devuelve 0 si la velocidad no esta en la lista.
static speed_t velocidad(int baudios)
{
    switch (baudios) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return 0;
    }
}

// Abre el puerto y lo deja en modo "crudo": los bytes pasan tal cual, sin que el sistema
// interprete fines de linea ni caracteres de control (cfmakeraw). CLOCAL ignora las lineas de
// modem, CREAD habilita la recepcion y se quitan el segundo bit de parada (CSTOPB) y el control
// de flujo por hardware (CRTSCTS). Al final se vacian los buffers (tcflush) para no leer
// basura que quedara de antes.
int serie_abrir(const char *ruta, int baudios)
{
    const speed_t v = velocidad(baudios);
    if (v == 0) return -1;
    const int fd = open(ruta, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CSTOPB | CRTSCTS);
    cfsetispeed(&t, v);
    cfsetospeed(&t, v);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// Escribe todo el bloque. Como el puerto es no bloqueante, write puede aceptar solo una parte
// o devolver EAGAIN ("ahora no"); se reintenta hasta que sale todo o hay un error real.
int serie_escribir(int fd, const uint8_t *datos, int n)
{
    int hechos = 0;
    while (hechos < n) {
        const ssize_t r = write(fd, datos + hechos, (size_t)(n - hechos));
        if (r > 0) hechos += (int)r;
        else if (r < 0 && errno != EAGAIN && errno != EINTR) return -1;
    }
    return hechos;
}

// Lee lo que haya llegado, sin esperar: EAGAIN significa "no hay nada" y se devuelve 0.
int serie_leer(int fd, uint8_t *buf, int cap)
{
    const ssize_t r = read(fd, buf, (size_t)cap);
    if (r >= 0) return (int)r;
    return errno == EAGAIN || errno == EINTR ? 0 : -1;
}

// Cierra el puerto (si estaba abierto).
void serie_cerrar(int fd)
{
    if (fd >= 0) close(fd);
}

#endif

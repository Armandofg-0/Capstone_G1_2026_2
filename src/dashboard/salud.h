// Salud del equipo para el panel del dashboard: CPU por nucleo, memoria, temperatura y
// estado de throttling. Se lee desde el hilo del dashboard, nunca desde el del lazo.
#ifndef SALUD_H
#define SALUD_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SALUD_NUCLEOS_MAX 16

typedef struct {
    int n_nucleos;
    double cpu[SALUD_NUCLEOS_MAX];  // % de uso por nucleo (en Windows, el total en cpu[0])
    double mem_usada_mb, mem_total_mb, temp_c;
    // Mascara de get_throttled, leida de /sys/devices/platform/soc/soc:firmware/get_throttled
    // (hexadecimal). Esa ruta queda por confirmar en la Pi. No se lanzan procesos (vcgencmd).
    int throttled;
} Salud;           // temp_c = NaN y throttled = -1 si no esta disponible

typedef struct {
    unsigned long long previo[SALUD_NUCLEOS_MAX][2];  // {total, ocioso} de la lectura anterior
    int hay_previo;
} MedidorSalud;

void salud_iniciar(MedidorSalud *m);
// La CPU (%) es la del intervalo desde la lectura anterior; la primera lectura da el promedio
// desde el arranque del sistema.
// Solo lee archivos de /proc y /sys (o llama a la API de Windows): nada bloqueante.
void salud_leer(MedidorSalud *m, Salud *s);
// JSON en una linea; NaN y no disponible (throttled < 0) salen como null. Devuelve el largo,
// o -1 si no cabe en cap.
int salud_json(const Salud *s, char *dst, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* SALUD_H */

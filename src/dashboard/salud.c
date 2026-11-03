// Salud de la maquina (ver salud.h): uso de CPU por nucleo, memoria, temperatura y si la
// Raspberry Pi esta bajando su frecuencia por calor o por falta de voltaje ("throttling").
// Se muestra en el dashboard. En Linux se lee de archivos especiales del sistema (/proc y
// /sys), que el kernel actualiza solo; en Windows (el PC de desarrollo) se usan llamadas de
// su API y no hay temperatura.
#include "salud.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

// Deja el medidor en cero (memset llena la estructura completa con bytes 0).
void salud_iniciar(MedidorSalud *m) { memset(m, 0, sizeof *m); }

// Porcentaje de uso del intervalo; sin intervalo (el total no avanza) da 0.
static double porcentaje(unsigned long long total, unsigned long long ocioso,
                         const unsigned long long previo[2]) {
    if (total <= previo[0]) return 0.0;
    double dt = (double)(total - previo[0]);
    double di = ocioso >= previo[1] ? (double)(ocioso - previo[1]) : 0.0;
    double p = 100.0 * (1.0 - di / dt);
    return p < 0.0 ? 0.0 : p > 100.0 ? 100.0 : p;
}

#ifdef _WIN32

// --- Version Windows ---
// Windows entrega los tiempos como FILETIME: un numero de 64 bits partido en dos mitades de
// 32. Esta funcion las vuelve a juntar.
static unsigned long long ft64(FILETIME f) {
    return ((unsigned long long)f.dwHighDateTime << 32) | f.dwLowDateTime;
}

void salud_leer(MedidorSalud *m, Salud *s) {
    s->n_nucleos = 1;
    memset(s->cpu, 0, sizeof s->cpu);
    FILETIME ocioso, nucleo, usuario;
    if (GetSystemTimes(&ocioso, &nucleo, &usuario)) {
        // El tiempo de nucleo ya incluye el ocioso.
        unsigned long long id = ft64(ocioso), tot = ft64(nucleo) + ft64(usuario);
        s->cpu[0] = porcentaje(tot, id, m->previo[0]);
        m->previo[0][0] = tot;
        m->previo[0][1] = id;
        m->hay_previo = 1;
    }
    MEMORYSTATUSEX ms;
    ms.dwLength = sizeof ms;
    if (GlobalMemoryStatusEx(&ms)) {
        s->mem_total_mb = (double)ms.ullTotalPhys / 1048576.0;
        s->mem_usada_mb = (double)(ms.ullTotalPhys - ms.ullAvailPhys) / 1048576.0;
    } else {
        s->mem_total_mb = s->mem_usada_mb = 0.0;
    }
    s->temp_c = NAN;
    s->throttled = -1;
}

#else

// --- Version Linux ---
// Uso de CPU por nucleo. /proc/stat trae, por nucleo, cuanto tiempo acumulado ha pasado en
// cada estado desde que encendio la maquina. El uso del intervalo sale de comparar con la
// lectura anterior (por eso se guarda "previo").
static void leer_cpu(MedidorSalud *m, Salud *s) {
    s->n_nucleos = 0;
    memset(s->cpu, 0, sizeof s->cpu);
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return;
    char linea[256];
    while (fgets(linea, sizeof linea, f)) {
        int id;
        unsigned long long v[8] = {0};
        // Lineas "cpuN": usuario nice sistema ocioso espera irq softirq robo.
        if (strncmp(linea, "cpu", 3) != 0 || linea[3] < '0' || linea[3] > '9') continue;
        if (sscanf(linea, "cpu%d %llu %llu %llu %llu %llu %llu %llu %llu", &id, &v[0], &v[1],
                   &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) < 5)
            continue;
        if (s->n_nucleos >= SALUD_NUCLEOS_MAX) break;
        int i = s->n_nucleos++;
        unsigned long long tot = 0;
        for (int j = 0; j < 8; j++) tot += v[j];
        unsigned long long ocioso = v[3] + v[4];
        s->cpu[i] = porcentaje(tot, ocioso, m->previo[i]);
        m->previo[i][0] = tot;
        m->previo[i][1] = ocioso;
    }
    fclose(f);
    m->hay_previo = 1;
}

// Memoria total y en uso, desde /proc/meminfo. "Disponible" (MemAvailable) es lo que de verdad
// podrian usar los programas; lo usado se calcula como total - disponible.
static void leer_memoria(Salud *s) {
    s->mem_total_mb = s->mem_usada_mb = 0.0;
    FILE *f = fopen("/proc/meminfo", "r");
    if (!f) return;
    char linea[128];
    double total = -1, disp = -1;
    unsigned long long kb;
    while (fgets(linea, sizeof linea, f)) {
        if (sscanf(linea, "MemTotal: %llu kB", &kb) == 1) total = (double)kb;
        else if (sscanf(linea, "MemAvailable: %llu kB", &kb) == 1) disp = (double)kb;
    }
    fclose(f);
    if (total < 0 || disp < 0) return;
    s->mem_total_mb = total / 1024.0;
    s->mem_usada_mb = (total - disp) / 1024.0;
}

// Temperatura de la CPU: el kernel la publica en milesimas de grado.
static double leer_temperatura(void) {
    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return NAN;
    long milic;
    int ok = fscanf(f, "%ld", &milic) == 1;
    fclose(f);
    return ok ? milic / 1000.0 : NAN;
}

// Solo sysfs, sin lanzar procesos: el hilo del dashboard atiende los sockets del video.
static int leer_throttled(void) {
    FILE *f = fopen("/sys/devices/platform/soc/soc:firmware/get_throttled", "r");
    if (!f) return -1;
    char b[32];
    char *fin = NULL;
    unsigned long v = 0;
    if (fgets(b, sizeof b, f)) v = strtoul(b, &fin, 16);
    fclose(f);
    return (fin && fin != b) ? (int)v : -1;
}

void salud_leer(MedidorSalud *m, Salud *s) {
    leer_cpu(m, s);
    leer_memoria(s);
    s->temp_c = leer_temperatura();
    s->throttled = leer_throttled();
}

#endif

// Escribe un numero en formato JSON. JSON no admite NaN ni infinito, asi que esos valores
// (por ejemplo, la temperatura en Windows) se escriben como null.
static void real_json(char *dst, size_t cap, double v) {
    if (isfinite(v)) snprintf(dst, cap, "%.6g", v);
    else snprintf(dst, cap, "null");
}

// Arma el texto JSON con toda la salud, para enviarlo al navegador. Devuelve el largo
// escrito, o -1 si no cupo en dst.
int salud_json(const Salud *s, char *dst, size_t cap) {
    if (cap == 0) return -1;
    size_t n = 0;
    int w;
    char b[40];
// AGREGAR es una macro (un "copiar y pegar" que hace el preprocesador): agrega texto al final
// de dst y, si no cabe, deja dst vacio y sale de la funcion con -1. El do { ... } while (0)
// es el truco habitual para que una macro de varias lineas se comporte como una sola
// instruccion. Se borra con #undef al terminar la funcion.
#define AGREGAR(...)                                          \
    do {                                                      \
        w = snprintf(dst + n, cap - n, __VA_ARGS__);          \
        if (w < 0 || (size_t)w >= cap - n) {                  \
            dst[0] = '\0';                                    \
            return -1;                                        \
        }                                                     \
        n += (size_t)w;                                       \
    } while (0)
    int nn = s->n_nucleos < 0 ? 0 : s->n_nucleos > SALUD_NUCLEOS_MAX ? SALUD_NUCLEOS_MAX : s->n_nucleos;
    AGREGAR("{\"n_nucleos\":%d,\"cpu\":[", nn);
    for (int i = 0; i < nn; i++) {
        real_json(b, sizeof b, s->cpu[i]);
        AGREGAR("%s%s", i ? "," : "", b);
    }
    AGREGAR("],\"mem_usada_mb\":");
    real_json(b, sizeof b, s->mem_usada_mb);
    AGREGAR("%s,\"mem_total_mb\":", b);
    real_json(b, sizeof b, s->mem_total_mb);
    AGREGAR("%s,\"temp_c\":", b);
    real_json(b, sizeof b, s->temp_c);
    AGREGAR("%s,\"throttled\":", b);
    if (s->throttled < 0) AGREGAR("null}");
    else AGREGAR("%d}", s->throttled);
#undef AGREGAR
    return (int)n;
}

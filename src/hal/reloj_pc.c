// Reloj monotonico (ver reloj_pc.h). En Windows, QueryPerformanceCounter; en los demas
// sistemas, clock_gettime con CLOCK_MONOTONIC.
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L
#endif
#include "reloj_pc.h"

// reloj_pc_ms: milisegundos de un reloj "monotonico", que nunca retrocede aunque se cambie la
// hora del sistema. Sirve para medir intervalos, no la hora del dia.
#ifdef _WIN32
#include <windows.h>
double reloj_pc_ms(void)
{
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart)
        QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return 1e3 * (double)c.QuadPart / (double)f.QuadPart;
}
#else
#include <time.h>
double reloj_pc_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1e3 * ts.tv_sec + 1e-6 * ts.tv_nsec;
}
#endif

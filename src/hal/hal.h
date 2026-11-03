// Capa de abstraccion de hardware (CONTEXTO_PROYECTO.md sec. 5.1).
//
// Cuatro interfaces, cada una con una implementacion simulada (ahora) y una real (despues).
// El estimador y el controlador no saben cual usan. Se escriben como structs con punteros a
// funcion y un contexto opaco, para que el nucleo siga en C portable.
//
// Tiempos: enteros en microsegundos. Cada reloj tiene su propia base (perdida.tex, seccion
// "Base de tiempo comun"): la Pi usa CLOCK_BOOTTIME, que es el de los timestamps de la
// camara, y la ESP32 usa su tick tau, que es la base de tiempo del lazo. Pasar de uno a otro
// es trabajo del estimador de sincronizacion, no de estas interfaces.
#ifndef HAL_H
#define HAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Reloj --------------------------------------------------------------------------------
// Hora local de un dispositivo, en us. En simulacion, un reloj con desfase theta y deriva eps
// respecto del tiempo verdadero: tau = (1 + eps) t + theta.
typedef struct {
    void *ctx;
    int64_t (*ahora_us)(void *ctx);
} Reloj;

// --- Camara -------------------------------------------------------------------------------
// Nivel de medicion (el nivel de imagen, para probar la vision, vive en C++ junto a la
// escena). Cada cuadro trae su SensorTimestamp y, si hubo deteccion, la fila donde esta el
// blanco, con lo que el estimador calcula el instante de medicion
// t_m = t_SOF + v t_fila + t_e/2 (obturador rodante).
typedef struct {
    int64_t t_sof_us;  // SensorTimestamp: inicio de la exposicion de la primera fila (reloj Pi)
    double t_fila_us;  // tiempo de lectura de una fila
    double t_e_us;     // tiempo de exposicion
    int hay_deteccion;
    double z[2];       // pixel medido (px)
    double sigma;      // incertidumbre que reporta la percepcion (px)
    double tamano;     // tamano aparente (px; solo perfil avion, NaN si no aplica)
} MedicionCamara;

typedef struct {
    void *ctx;
    // Espera (o, en simulacion, genera) el cuadro siguiente. Devuelve 0 si no hay mas.
    int (*siguiente)(void *ctx, MedicionCamara *m);
} Camara;

// --- Enlace -------------------------------------------------------------------------------
// Canal de bytes Pi <-> ESP32 (UART). En simulacion: latencia, jitter y perdidas de
// Gilbert-Elliott; real: la UART. Cada extremo tiene su propio Enlace.
typedef struct {
    void *ctx;
    // Encola n bytes para enviar en el instante local t_us.
    void (*enviar)(void *ctx, const uint8_t *datos, size_t n, int64_t t_us);
    // Copia en buf los bytes que llegaron hasta el instante local t_us (como maximo cap).
    // Devuelve cuantos copio.
    size_t (*recibir)(void *ctx, uint8_t *buf, size_t cap, int64_t t_us);
} Enlace;

// --- Ejecutor -----------------------------------------------------------------------------
// La ESP32 vista desde el arnes de simulacion: ejecutor determinista de planes a 1 kHz
// (seleccion de paquete por t_a, interpolacion de velocidad, cola, frenado, limitador, DDA y
// observador articular). Lee los planes y escribe la telemetria por su Enlace. En el
// sistema real corre sola en la ESP32 y la Pi solo la ve a traves del Enlace.
typedef struct {
    void *ctx;
    // Avanza el ejecutor (y, en simulacion, los motores) hasta el tick tau_us.
    void (*avanzar)(void *ctx, int64_t tau_us);
} Ejecutor;

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */

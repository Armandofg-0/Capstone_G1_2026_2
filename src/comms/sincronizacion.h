// Base de tiempo comun Pi <-> ESP32 (perdida.tex, "Base de tiempo comun").
//
// La base del lazo es el tick de la ESP32, tau (us). La Pi mapea su reloj t (CLOCK_BOOTTIME,
// en s, el de los timestamps de la camara) con el modelo tau = (1 + eps) t + theta. En cada
// intercambio (t1, tau2, tau3, t4), como en la sincronizacion probabilistica de Cristian:
//   theta_hat = 1/2 [(tau2 - t1) + (tau3 - t4)],  delta = (t4 - t1) - (tau3 - tau2),  |theta_hat - theta| <= delta/2.
// Se conservan los intercambios con delta cerca del minimo de la ventana (donde la planificacion
// de Linux no agrego demora) y se ajustan (theta, eps) por minimos cuadrados en una ventana
// deslizante.
//
// Cristian supone un camino simetrico. La parte asimetrica conocida, el tiempo de
// transmision de cada mensaje (el plan es mucho mas largo que la telemetria), se compensa
// antes: t1 y tau3 se refieren al FIN de la transmision de su mensaje (t1 + bytes*10/baudios),
// igual que tau2 y t4, que se marcan al terminar de recibir. Sin eso, el sesgo es la mitad de
// la diferencia de duraciones (~0.5 ms con un plan de 127 bytes y una telemetria de 35).
#ifndef SINCRONIZACION_H
#define SINCRONIZACION_H

#ifdef __cplusplus
extern "C" {
#endif

#define SYNC_MAX 1024

typedef struct {
    double t1;   // fin del envio desde la Pi (s, reloj de la Pi)
    double tau2; // recepcion en la ESP32 (us, tick)
    double tau3; // fin del envio de la respuesta de la ESP32 (us, tick)
    double t4;   // recepcion en la Pi (s)
} IntercambioSync;

typedef struct {
    double ventana; // s
    double margen;  // se conservan los intercambios con delta <= delta_min + margen (s)
    int n;
    double t_mid[SYNC_MAX], theta[SYNC_MAX], delta[SYNC_MAX]; // s, us, s
    // Resultado: tau = (1 + eps) t 1e6 + theta, con t en s y tau y theta en us.
    double theta_us, eps;
    int valido;
} Sincronizacion;

void sync_iniciar(Sincronizacion *s, double ventana, double margen);
void sync_agregar(Sincronizacion *s, const IntercambioSync *x);
double sync_tau_de_t(const Sincronizacion *s, double t);    // us
double sync_t_de_tau(const Sincronizacion *s, double tau);  // s

#ifdef __cplusplus
}
#endif

#endif /* SINCRONIZACION_H */

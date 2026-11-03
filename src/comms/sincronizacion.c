// Base de tiempo comun (ver sincronizacion.h).
#include "sincronizacion.h"

#include <math.h>

// Arranca la sincronizacion sin intercambios: hasta tener datos se supone que los relojes
// coinciden.
void sync_iniciar(Sincronizacion *s, double ventana, double margen)
{
    s->ventana = ventana;
    s->margen = margen;
    s->n = 0;
    s->theta_us = 0.0;
    s->eps = 0.0;
    s->valido = 0;
}
//---------------------------------------------------------------------------
// Agrega un intercambio (t1 Pi envia, tau2 ESP32 recibe, tau3 ESP32 envia, t4 Pi recibe),
// descarta los que sufrieron demora extra y reajusta la recta que relaciona ambos relojes.
void sync_agregar(Sincronizacion *s, const IntercambioSync *x)
{
    const double t_mid = 0.5 * (x->t1 + x->t4);
    const double theta = 0.5 * ((x->tau2 - 1e6 * x->t1) + (x->tau3 - 1e6 * x->t4));
    const double delta = (x->t4 - x->t1) - 1e-6 * (x->tau3 - x->tau2);
    // Ventana deslizante: se descartan los intercambios viejos (o los mas antiguos si se llena).
    int ini = 0;
    while (ini < s->n && (t_mid - s->t_mid[ini] > s->ventana || s->n - ini >= SYNC_MAX))
        ini++;
    int k = 0;
    for (int i = ini; i < s->n; ++i, ++k) {
        s->t_mid[k] = s->t_mid[i];
        s->theta[k] = s->theta[i];
        s->delta[k] = s->delta[i];
    }
    s->t_mid[k] = t_mid;
    s->theta[k] = theta;
    s->delta[k] = delta;
    s->n = k + 1;

    // Filtro de minimo retardo y ajuste lineal theta_hat_i = theta + eps 1e6 t_i (centrado en t para que
    // la regresion este bien condicionada).
    double dmin = INFINITY;
    for (int i = 0; i < s->n; ++i)
        if (s->delta[i] < dmin)
            dmin = s->delta[i];
    double st = 0.0, sy = 0.0;
    int m = 0;
    for (int i = 0; i < s->n; ++i)
        if (s->delta[i] <= dmin + s->margen) {
            st += s->t_mid[i];
            sy += s->theta[i];
            m++;
        }
    const double tm = st / m, ym = sy / m;
    double stt = 0.0, sty = 0.0;
    for (int i = 0; i < s->n; ++i)
        if (s->delta[i] <= dmin + s->margen) {
            stt += (s->t_mid[i] - tm) * (s->t_mid[i] - tm);
            sty += (s->t_mid[i] - tm) * (s->theta[i] - ym);
        }
    // Con menos de un segundo de datos la pendiente esta mal determinada: solo el desfase.
    const double pendiente = (m >= 3 && stt > 1.0) ? sty / stt : 0.0; // us por s
    s->eps = pendiente * 1e-6;
    s->theta_us = ym - pendiente * tm;
    s->valido = 1;
}
//---------------------------------------------------------------------------
// Traduce un instante del reloj de la Pi (s) al tick de la ESP32 (us)...
double sync_tau_de_t(const Sincronizacion *s, double t)
{
    return (1.0 + s->eps) * 1e6 * t + s->theta_us;
}
//---------------------------------------------------------------------------
// ... y al reves.
double sync_t_de_tau(const Sincronizacion *s, double tau)
{
    return (tau - s->theta_us) / ((1.0 + s->eps) * 1e6);
}

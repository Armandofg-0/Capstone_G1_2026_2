// Ritmo de envio del dashboard (ver ritmo.h): decide cuando toca mandar el siguiente
// paquete de datos para no superar una frecuencia fija (por ejemplo, 10 paneles por segundo).
// Es como un metronomo: guarda cuando "deberia" haber sonado el ultimo golpe y, a partir de
// ahi, calcula el siguiente.
#include "ritmo.h"

// Arranca el ritmo con un periodo dado. El "ultimo envio" se pone en un pasado muy lejano
// (-1e300 ms) para que el primer envio quede habilitado de inmediato.
void ritmo_iniciar(Ritmo *r, double periodo_ms) {
    r->periodo_ms = periodo_ms;
    r->t_ultimo = -1e300;
}

// Instante (ms) a partir del cual toca el proximo envio.
double ritmo_turno(const Ritmo *r) { return r->t_ultimo + r->periodo_ms; }

// Registra que se envio en el instante t. El turno siguiente avanza un periodo desde el
// anterior (asi el ritmo promedio se mantiene exacto), pero sin quedarse mas de un periodo
// atras de t: si el dashboard estuvo pausado, no se intenta "recuperar" con una rafaga.
void ritmo_enviado(Ritmo *r, double t) {
    const double siguiente = r->t_ultimo + r->periodo_ms, minimo = t - r->periodo_ms;
    r->t_ultimo = siguiente > minimo ? siguiente : minimo;
}

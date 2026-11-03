// Turno de envio del video a fps_max (publicador del dashboard). El plazo se ancla a una
// grilla de periodo 1/fps_max y no al instante del ultimo envio: anclarlo al envio sumaria a
// cada cuadro el atraso del anterior, y con una fuente de 30 fps con jitter y fps_max = 30
// los cuadros esperarian (o se descartarian) sin necesidad.
//
// Tras un envio en t, la grilla avanza un periodo, pero nunca queda mas de un periodo atras
// de t: t_ultimo = max(t_ultimo + periodo, t - periodo). Asi la rafaga es de a lo mas dos
// cuadros (uno atrasado y el de su turno) y la tasa media no pasa de fps_max.
#ifndef RITMO_H
#define RITMO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double periodo_ms;
    double t_ultimo;  // ultimo punto de la grilla usado (ms, reloj_pc_ms)
} Ritmo;

void ritmo_iniciar(Ritmo *r, double periodo_ms);  // el primer cuadro sale cuando llegue
// Instante desde el que el proximo cuadro tiene turno.
double ritmo_turno(const Ritmo *r);
// Registra un envio en t (t >= ritmo_turno(r)).
void ritmo_enviado(Ritmo *r, double t);

#ifdef __cplusplus
}
#endif

#endif /* RITMO_H */

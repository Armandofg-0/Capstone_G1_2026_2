// Perdidas de paquetes de Gilbert-Elliott (perdida.tex, "Que se pierde realmente"): cadena de
// Markov de dos estados con
//   p_GE = P{perdida | perdida},  q_GE = P{exito | exito},
//   rafaga media = 1/(1 - p_GE),  tasa de perdidas = (1 - q_GE)/(2 - p_GE - q_GE).
// El caso i.i.d. es p_GE = 1 - q_GE.
//
// El canal sirve para la simulacion; el monitor corre en la Pi con los registros de llegada:
// p_hat = n_PP / n_P, q_hat = n_EE / n_E, y vigila el margen 1 - p_hat gamma de Quevedo-Nesic.
#ifndef GILBERT_ELLIOTT_H
#define GILBERT_ELLIOTT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double ge_p, ge_q;
    int perdiendo; // estado actual de la cadena
} CanalGE;

void ge_iniciar(CanalGE *c, double ge_p, double ge_q);
// Avanza la cadena un paquete con un uniforme u en (0, 1]; devuelve 1 si se pierde.
int ge_perdido(CanalGE *c, double u);

typedef struct {
    long n_P, n_PP, n_E, n_EE;
    int hay_anterior, anterior;
} MonitorGE;

void ge_monitor_iniciar(MonitorGE *m);
void ge_monitor_registrar(MonitorGE *m, int perdido);
double ge_monitor_p(const MonitorGE *m);
double ge_monitor_q(const MonitorGE *m);
double ge_monitor_margen(const MonitorGE *m, double gamma);

#ifdef __cplusplus
}
#endif

#endif /* GILBERT_ELLIOTT_H */

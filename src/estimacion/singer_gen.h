// Transicion del modelo de Singer para un intervalo arbitrario (un eje), generada por
// codegen/singer.py (sympy): F = exp(A Delta t) y Q = int exp(A s) B q_c B^T exp(A s)^T ds, con
// q_c = 2 sigma_m^2 / tau_m. Estado [posicion, velocidad, aceleracion]; matrices 3x3 por filas.
// Sirve para la prediccion con el intervalo real entre timestamps (informe, "Estado y
// dinamica") y para llevar el filtro hasta t_a (perdida.tex, F_xi(t_a - t_m)).
#ifndef SINGER_GEN_H
#define SINGER_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

void singer_FQ(double dt, double tau_m, double sigma_m, double F[9], double Q[9]);

#ifdef __cplusplus
}
#endif

#endif /* SINGER_GEN_H */

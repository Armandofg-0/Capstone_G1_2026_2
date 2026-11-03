// Condicion de estabilidad con perdidas markovianas de Quevedo y Nesic (perdida.tex,
// "Condicion de estabilidad con perdidas markovianas").
//
// Para el error de un eje con costo terminal LQR, el doble integrador A = [1 T; 0 1],
// B = [T^2/2; T], costo de etapa e^T Q e + R u^2, Q = diag(q_pos, q_vel), y P de la DARE:
//   gamma = lambda_max(P^{-1/2} A^T P A P^{-1/2})   (crecimiento con la entrada por defecto),
//   l = lambda_min(P^{-1/2} (Q + K^T R K) P^{-1/2}),
// y la exigencia p_GE gamma < 1. gamma debe calcularse para el costo del controlador real (C12 de
// la fase 0): con el costo de imagen W_s (px^2) y L_eff ~ f, q_pos = f^2 W_s.
#ifndef QUEVEDO_H
#define QUEVEDO_H

#ifdef __cplusplus
extern "C" {
#endif

// Devuelve 0 si la DARE no converge.
int quevedo_eje(double Ts, double q_pos, double q_vel, double R, double *gamma, double *ell);

#ifdef __cplusplus
}
#endif

#endif /* QUEVEDO_H */

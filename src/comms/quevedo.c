// Condicion de Quevedo-Nesic (ver quevedo.h). Todo es 2x2: DARE por iteracion de Riccati y
// valores propios generalizados por la cuadratica caracteristica.
#include "quevedo.h"

#include <math.h>

// Valores propios de M respecto de P (simetricas, P definida positiva): det(M - lambda P) = 0.
static void propios_gen(const double M[4], const double P[4], double *lmin, double *lmax)
{
    const double a = P[0] * P[3] - P[1] * P[2];
    const double b = -(M[0] * P[3] + M[3] * P[0] - M[1] * P[2] - M[2] * P[1]);
    const double c = M[0] * M[3] - M[1] * M[2];
    const double disc = sqrt(fmax(0.0, b * b - 4.0 * a * c));
    *lmin = (-b - disc) / (2.0 * a);
    *lmax = (-b + disc) / (2.0 * a);
}
//---------------------------------------------------------------------------
// Calcula gamma y l para un eje (ver quevedo.h): con ellos se sabe si el lazo sigue siendo
// estable aunque se pierdan paquetes con la probabilidad medida del canal. Primero se resuelve
// la ecuacion de Riccati discreta (DARE) iterando hasta que P deja de cambiar.
int quevedo_eje(double Ts, double q_pos, double q_vel, double R, double *gamma, double *ell)
{
    const double A[4] = {1.0, Ts, 0.0, 1.0}, B[2] = {0.5 * Ts * Ts, Ts};
    const double Q[4] = {q_pos, 0.0, 0.0, q_vel};
    double P[4] = {Q[0] + 1.0, 0.0, 0.0, Q[3] + 1.0}, K[2] = {0.0, 0.0};
    int ok = 0;
    for (int it = 0; it < 200000; ++it) {
        // K = (R + B^T P B)^-1 B^T P A;  P' = A^T P A + Q - A^T P B K.
        const double PB[2] = {P[0] * B[0] + P[1] * B[1], P[2] * B[0] + P[3] * B[1]};
        const double s = R + B[0] * PB[0] + B[1] * PB[1];
        const double BPA[2] = {PB[0] * A[0] + PB[1] * A[2], PB[0] * A[1] + PB[1] * A[3]};
        K[0] = BPA[0] / s;
        K[1] = BPA[1] / s;
        double PA[4], AtPA[4];
        PA[0] = P[0] * A[0] + P[1] * A[2];
        PA[1] = P[0] * A[1] + P[1] * A[3];
        PA[2] = P[2] * A[0] + P[3] * A[2];
        PA[3] = P[2] * A[1] + P[3] * A[3];
        AtPA[0] = A[0] * PA[0] + A[2] * PA[2];
        AtPA[1] = A[0] * PA[1] + A[2] * PA[3];
        AtPA[2] = A[1] * PA[0] + A[3] * PA[2];
        AtPA[3] = A[1] * PA[1] + A[3] * PA[3];
        double Pn[4];
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                Pn[2 * i + j] = AtPA[2 * i + j] + Q[2 * i + j] - BPA[i] * K[j];
        double cambio = 0.0;
        for (int i = 0; i < 4; ++i) {
            cambio = fmax(cambio, fabs(Pn[i] - P[i]) / (fabs(P[i]) + 1e-12));
            P[i] = Pn[i];
        }
        if (cambio < 1e-13) {
            ok = 1;
            break;
        }
    }
    if (!ok)
        return 0;
    double PA[4], AtPA[4], l1, l2;
    PA[0] = P[0] * A[0] + P[1] * A[2];
    PA[1] = P[0] * A[1] + P[1] * A[3];
    PA[2] = P[2] * A[0] + P[3] * A[2];
    PA[3] = P[2] * A[1] + P[3] * A[3];
    AtPA[0] = A[0] * PA[0] + A[2] * PA[2];
    AtPA[1] = A[0] * PA[1] + A[2] * PA[3];
    AtPA[2] = A[1] * PA[0] + A[3] * PA[2];
    AtPA[3] = A[1] * PA[1] + A[3] * PA[3];
    propios_gen(AtPA, P, &l1, gamma);
    const double QK[4] = {Q[0] + R * K[0] * K[0], R * K[0] * K[1], R * K[1] * K[0], Q[3] + R * K[1] * K[1]};
    propios_gen(QK, P, ell, &l2);
    return 1;
}

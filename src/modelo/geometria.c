// Inversa de la proyeccion h (ver geometria.h).
//
// El rayo del pixel en el marco de la camara es d = [x; y; 1], con x, y las coordenadas
// normalizadas. Del informe, ^cP = (a - rho ^cr)/rho con ^cr = [l_x; 0; 0] y a = R^T(q) u(psi)
// unitario. Como ^cP es paralelo a d, a = rho ^cr + kappa d con kappa > 0, y |a| = 1 da la cuadratica
//     kappa^2 (1 + x^2 + y^2) + 2 kappa rho l_x x + (rho l_x)^2 - 1 = 0,
// de la que se toma la raiz positiva. Despues u = R(q) a y psi sale de las esfericas.
#include "geometria.h"

#include "model.h"

// Inversa de la camara: dado un pixel s, la pose q y la distancia inversa rho, calcula la
// direccion psi del blanco vista desde la base. Se usa para arrancar el EKF con la primera
// deteccion. Devuelve 0 si el pixel no tiene solucion (por ejemplo, rayo hacia atras).
int geo_inversa(const Geometria *g, const double q[2], const double s[2], double rho, double psi[2])
{
    const double x = (s[0] - g->cu) / g->fu, y = (s[1] - g->cv) / g->fv;
    const double rl = rho * g->lx;
    const double A = 1.0 + x * x + y * y, B = rl * x, C = rl * rl - 1.0;
    const double disc = B * B - A * C;
    if (C >= 0.0 || disc < 0.0)
        return 0;
    const double k = (-B + sqrt(disc)) / A;
    const double a[3] = {rl + k * x, k * y, k};
    double R[9], u[3];
    R_0c(q[0], q[1], R);
    for (int i = 0; i < 3; ++i)
        u[i] = R[3 * i] * a[0] + R[3 * i + 1] * a[1] + R[3 * i + 2] * a[2];
    psi[0] = atan2(u[1], u[0]);
    psi[1] = asin(u[2] > 1.0 ? 1.0 : (u[2] < -1.0 ? -1.0 : u[2]));
    return 1;
}

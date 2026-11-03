
// Las funciones que se llaman en cada periodo de control (como L_eff o camera_normalize)
// se definen directamente aqui, en el header, marcadas como "static inline". Asi el
// compilador puede copiar su codigo directo dentro del lazo del MPC en vez de hacer una
// llamada a funcion, lo que ahorra tiempo cuando se llaman muchas veces.
#ifndef MODEL_H
#define MODEL_H

#include <math.h>

// Algunos compiladores (por ejemplo, el de Microsoft) no definen M_PI por defecto, asi que
// si no existe se define aqui a mano.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Parametros de una camara "pinhole". Con ellos se puede pasar de un punto en 3D a un pixel.
typedef struct {
    // Distancias focales expresadas en pixeles, una para cada eje de la imagen. Dicen
    // cuantos pixeles se desplaza un punto por cada unidad de desplazamiento angular.
    double fu, fv;

    // Punto principal: el pixel donde el eje optico de la camara atraviesa la imagen.
    // En una camara ideal es el centro exacto de la imagen.
    double cu, cv;

    int width, height;  // resolucion de la imagen (px)
} Intrinsecos;

// Devuelve s*, la posicion donde se quiere ver al objeto en la imagen. Se elige el punto
// principal porque ahi el objeto queda exactamente sobre el eje optico, es decir, la
// camara apunta directo al objeto.
static inline void camera_s_star(const Intrinsecos *cam, double s_star[2])
{
    s_star[0] = cam->cu;
    s_star[1] = cam->cv;
}

// Pasa de coordenadas en pixeles (s) a coordenadas normalizadas (x).
static inline void camera_normalize(const Intrinsecos *cam, const double s[2], double x[2])
{
    x[0] = (s[0] - cam->cu) / cam->fu;
    x[1] = (s[1] - cam->cv) / cam->fv;
}

// Lo contrario de la anterior: pasa de coordenadas normalizadas a pixeles.
static inline void camera_denormalize(const Intrinsecos *cam, const double x[2], double s[2])
{
    s[0] = cam->cu + cam->fu * x[0];
    s[1] = cam->cv + cam->fv * x[1];
}

// Proyecta un punto 3D p_c (expresado en el sistema de coordenadas de la camara, donde el
// eje z apunta hacia adelante) sobre la imagen, y deja el pixel resultante en s.
static inline int camera_project(const Intrinsecos *cam, const double p_c[3], double s[2])
{
    if (p_c[2] <= 1e-9)
        return 0;
    const double inv_z = 1.0 / p_c[2]; // se divide una sola vez y luego se multiplica
    s[0] = cam->cu + cam->fu * (p_c[0] * inv_z);
    s[1] = cam->cv + cam->fv * (p_c[1] * inv_z);
    return 1;
}

// Matrices de rotacion de 3x3 alrededor del eje x y del eje z, en un angulo "a" (rad).
void Rx(double a, double R[9]);
void Rz(double a, double R[9]);


void R_0c(double theta_p, double theta_t, double R[9]);
void R_0c_T_apply(double theta_p, double theta_t, const double p_0[3], double p_c[3]);
void bearing_to_unit(double psi, double phi, double u[3]);
void J_geo(double theta_t, double J[6]);
void L_omega(double x, double y, double L[6]);

// Jacobiano efectivo de imagen (2x2).
//
// Se obtiene de multiplicar tres cosas, que aca ya vienen multiplicadas a mano:
//     L = diag(fu, fv) * L_omega(x, y) * J_geo(theta_t)
// J_geo pasa de velocidades de la torreta a velocidad angular de la camara, L_omega pasa
// de esa velocidad angular a movimiento en coordenadas normalizadas, y diag(fu, fv) pasa
// de coordenadas normalizadas a pixeles.
static inline void L_eff(const Intrinsecos *cam, const double s[2], double theta_t, double L[4])
{
    // Se pasa el pixel a coordenadas normalizadas (igual que camera_normalize):
    const double x = (s[0] - cam->cu) / cam->fu;
    const double y = (s[1] - cam->cv) / cam->fv;
    const double ct = cos(theta_t), st = sin(theta_t);
    const double xy = x * y; // se reutiliza en dos lugares

    // Fila 0 (eje u): L[0] es el efecto del pan y L[1] el efecto del tilt.
    L[0] = cam->fu * ((1.0 + x * x) * ct + y * st);
    L[1] = cam->fu * xy;
    // Fila 1 (eje v): L[2] es el efecto del pan y L[3] el efecto del tilt.
    L[2] = cam->fv * (xy * ct - x * st);
    L[3] = cam->fv * (1.0 + y * y);
}

#ifdef __cplusplus
}
#endif

#endif /* MODEL_H */

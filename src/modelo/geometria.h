// Geometria de camara y torreta con offset del centro optico (informe_control.tex,
// "Modelo propuesto").
//
// El blanco se describe desde p0 (la interseccion de los ejes) con su azimut lambda y elevacion
// mu, psi = [lambda; mu], y su distancia inversa rho = 1/r. La pose de la torreta es q = [theta_p; theta_t].
// El centro optico esta desplazado l_x del eje de pan, en la direccion +x de la imagen. Con
// l_x = 0 se recupera la rotacion pura.
//
// Las funciones geo_* salvo geo_inversa estan en geometria_gen.c, GENERADO por
// codegen/geometria.py (sympy), que ademas verifica las formulas contra el informe.
#ifndef GEOMETRIA_H
#define GEOMETRIA_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double fu, fv; // focales (px)
    double cu, cv; // punto principal = s* (px)
    double lx;     // l_x: offset del centro optico respecto del eje de pan (m)
} Geometria;

// s = h(q, psi, rho) (px).
void geo_h(const Geometria *g, const double q[2], const double psi[2], double rho, double s[2]);

// H_q = dh/dq, H_psi = dh/dpsi (2x2, por filas) y H_rho = dh/drho (2x1).
void geo_jacobianos(const Geometria *g, const double q[2], const double psi[2], double rho,
                    double Hq[4], double Hpsi[4], double Hrho[2]);

// a_z: tercera componente de a = R^T(q) u(psi). La profundidad es Z = ^cP_z = a_z/rho, asi que
// 1/Z = rho/a_z y el blanco esta delante de la camara si a_z > 0.
double geo_az(const double q[2], const double psi[2]);

// Mapa de mirada q* = g(psi, rho): los angulos que dejan el blanco en s*, y dg/dpsi (2x2).
void geo_mirada(const Geometria *g, const double psi[2], double rho, double qs[2]);
void geo_mirada_jac(const Geometria *g, const double psi[2], double rho, double dg[4]);

// Tamano aparente h_w = k_w rho/a_z + b_w (perfil avion) y su gradiente respecto de
// [theta_p, theta_t, lambda, mu, rho].
double geo_tamano(const Geometria *g, const double q[2], const double psi[2], double rho,
                  double kw, double bw, double grad[5]);

// Rumbo r = s* + F_pi (a_x - rho l_x, a_y) (px): la proyeccion sin la division por a_z. Vale s*
// exactamente donde h = s*, coincide con h cerca del centro y es acotado lejos de el (sin la
// singularidad a_z -> 0). Lo usa el NMPC en el costo y en la FOV. Jacobianos como geo_jacobianos.
void geo_rumbo(const Geometria *g, const double q[2], const double psi[2], double rho, double r[2]);
void geo_rumbo_jac(const Geometria *g, const double q[2], const double psi[2], double rho,
                   double Rq[4], double Rpsi[4], double Rrho[2]);

// Inversa de h: direccion psi del blanco que se ve en el pixel s desde la pose q, con
// distancia inversa rho (arranque del filtro). Devuelve 0 si no hay solucion (rho l_x >= 1).
int geo_inversa(const Geometria *g, const double q[2], const double s[2], double rho, double psi[2]);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRIA_H */

// Traduccion a C de model.py.
// Recordemos que todas las matrices se guardan planas y por filas: en una matriz de 3x3, 
// R[0], R[1] y R[2] son la primera fila, R[3], R[4] y R[5] la segunda, y R[6], R[7] y R[8] la tercera. 
//Por eso las asignaciones se escriben de a tres por linea: asi se "ve" la forma de la matriz.
#include "model.h"

// Rotacion de un angulo "a" alrededor del eje x. El eje x no cambia (primera fila y
// columna son las de la identidad) y el plano y-z gira.
void Rx(double a, double R[9])
{
    const double c = cos(a), s = sin(a);
    R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
    R[3] = 0.0; R[4] = c;   R[5] = -s;
    R[6] = 0.0; R[7] = s;   R[8] = c;
}
//---------------------------------------------------------------------------
// Rotacion de un angulo "a" alrededor del eje z. Ahora es el eje z el que no cambia y el
// plano x-y el que gira. Es el giro que produce el pan de la torreta.
void Rz(double a, double R[9])
{
    const double c = cos(a), s = sin(a);
    R[0] = c;   R[1] = -s;  R[2] = 0.0;
    R[3] = s;   R[4] = c;   R[5] = 0.0;
    R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;
}
//---------------------------------------------------------------------------
// Orientacion de la camara respecto del mundo. Esta explicacion se encuentra dentro del informe, sin embargo
// se recalcan las siguientes ideas para entender bien todo: 
//  1) Rx(theta_t): primero se aplica el tilt, que gira la camara alrededor de su propio
//     eje x (el eje horizontal de la imagen), subiendo o bajando la vista.
//  2) R_REST = [[0,0,1],[-1,0,0],[0,-1,0]]: es la orientacion de "reposo" de la camara.
//     Traduce la convencion de la camara (x a la derecha de la imagen, y hacia abajo, z
//     hacia adelante) a la del mundo (x hacia adelante, y a la izquierda, z hacia arriba).
//  3) Rz(theta_p): por ultimo se aplica el pan, que gira todo alrededor del eje vertical.
//
// Todo este producto se expandio a mano y quedo lo siguiente:
void R_0c(double theta_p, double theta_t, double R[9])
{
    const double cp = cos(theta_p), sp = sin(theta_p);
    const double ct = cos(theta_t), st = sin(theta_t);
    R[0] = sp;  R[1] = cp * st; R[2] = cp * ct;
    R[3] = -cp; R[4] = sp * st; R[5] = sp * ct;
    R[6] = 0.0; R[7] = -ct;     R[8] = st;
}
//---------------------------------------------------------------------------
// Expresa en el sistema de la camara un punto p_0 que esta expresado en el sistema del
// mundo. Para eso hay que multiplicar por la transpuesta de R_0c (la transpuesta de una
// rotacion es su inversa, o sea, deshace el giro).
//
// Multiplicar por la transpuesta equivale a hacer el producto punto de p_0 con cada
// columna de R_0c.
void R_0c_T_apply(double theta_p, double theta_t, const double p_0[3], double p_c[3])
{
    const double cp = cos(theta_p), sp = sin(theta_p);
    const double ct = cos(theta_t), st = sin(theta_t);
    const double h = cp * p_0[0] + sp * p_0[1];   // componente comun a las filas 1 y 2
    p_c[0] = sp * p_0[0] - cp * p_0[1];
    p_c[1] = st * h - ct * p_0[2];
    p_c[2] = ct * h + st * p_0[2];
}
//---------------------------------------------------------------------------
// Convierte una direccion en un vector unitario. Es la conversion de coordenadas esfericas,
// a cartesianas con radio 1: cos(phi) es cuanto del vector queda acostado en el plano horizontal, 
// y eso se reparte entre x e y segun psi; sin(phi) es cuanto apunta hacia arriba.
void bearing_to_unit(double psi, double phi, double u[3])
{
    const double cf = cos(phi);
    u[0] = cf * cos(psi);
    u[1] = cf * sin(psi);
    u[2] = sin(phi);
}
//---------------------------------------------------------------------------
// Jacobiano geometrico J (3x2, aplanado en 6 elementos). Si qd = [velocidad de pan,
// velocidad de tilt], entonces la velocidad angular de la camara, expresada en los ejes de
// la propia camara es omega_c = J * qd (esta expresion esta bien desarrollada en el informe).

void J_geo(double theta_t, double J[6])
{
    J[0] = 0.0;            J[1] = 1.0;
    J[2] = -cos(theta_t);  J[3] = 0.0;
    J[4] = sin(theta_t);   J[5] = 0.0;
}
//---------------------------------------------------------------------------
// matriz de interaccion de un punto (2x3). Esta expresion esta bien definida en el informe.
void L_omega(double x, double y, double L[6])
{
    L[0] = x * y;       L[1] = -(1.0 + x * x); L[2] = y;
    L[3] = 1.0 + y * y; L[4] = -x * y;         L[5] = -x;
}

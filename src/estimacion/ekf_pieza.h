// Filtro de Kalman extendido en el marco de la pieza (informe_control.tex, "Filtro de Kalman
// en el marco de la pieza").
//
// Estado xi = [lambda, mu, lambda_dot, mu_dot, lambda_ddot, mu, rho]: direccion del blanco vista desde p0, sus derivadas y la
// distancia inversa rho = 1/r. Ninguna cambia cuando la torreta gira.
//
//  - Prediccion lineal con el modelo de Singer por eje, para el intervalo real entre
//    instantes: F = blkdiag(F1(Delta t) (x) I2, 1), Q = blkdiag(phi_k Q1(Delta t) (x) I2, q_rho Delta t). La camara no
//    aparece en la prediccion: lo que hace el blanco esta en la dinamica.
//  - Medicion no lineal z = h(q_hat(t_m), psi, rho) + nu con la pose en el instante de captura: lo que
//    hace la camara esta en la medicion. El ego-movimiento entra solo por aqui, nunca se
//    resta de z (sin doble conteo). R = sigma^2 I + H_q Sigma_q H_q^T: el error de pose entra a R.
//  - Perfil avion: medicion apilada con el tamano aparente z_w = k_w rho/a_z + b_w, y rho
//    estimado con cotas por pseudo-medicion. Demas perfiles: rho considerado (Schmidt-Kalman).
//  - Arranque de dos puntos, Joseph, puerta chi^2, phi_k adaptativo con el NIS de las aceptadas
//    contra la chi^2 truncada, inflacion de P al rechazar, y dos modos de Singer con seleccion
//    parametrizada: un modo fijo, conmutacion por NIS o IMM (Blom y Bar-Shalom).
//
// Sin memoria dinamica: todo vive en EKFPieza.
#ifndef EKF_PIEZA_H
#define EKF_PIEZA_H

#include "geometria.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_N 7
#define EKF_VENTANA_MAX 32

// Puertas chi^2 al 99 % y medias de la chi^2 truncada por ellas (referencia del NIS con gating).
#define EKF_CHI2_2 9.21
#define EKF_CHI2_3 11.34
#define EKF_NIS_REF_2 1.91
#define EKF_NIS_REF_3 2.89

typedef enum { RHO_CONSIDERADO = 0, RHO_ESTIMADO } TratamientoRho;
typedef enum { SELECCION_FIJO = 0, SELECCION_NIS, SELECCION_IMM } SeleccionModo; // [ABIERTA]

typedef struct {
    // Modos de Singer (rad/s^2 y s): el 0 es el principal (reposo en el laser) y el 1 el
    // alternativo (barrido). Con SELECCION_FIJO solo se usa el modo 0.
    double tau_m[2], sigma_m[2];
    int seleccion;               // SeleccionModo
    double nis_subir, nis_bajar; // histeresis sobre la media del NIS para pasar al modo 1 y volver
    double imm_T[2];             // SELECCION_IMM: permanencia media en cada modo (s)
    // rho = 1/r.
    int tratamiento_rho;         // TratamientoRho
    double rho0, sigma_rho0;     // valor inicial y su desviacion (1/m)
    double q_rho;                // densidad del paseo aleatorio (1/m^2 por s); 0 si es considerado
    double rho_min, rho_max, sigma_rho_cota; // cotas por pseudo-medicion (solo estimado)
    // Tamano aparente (perfil avion).
    int usar_tamano;
    double kw, bw, sigma_w;
    double sigma_q;              // incertidumbre de la pose q_hat (rad), entra a R
    int adaptativo;              // phi_k segun el NIS
    int ventana;                 // mediciones aceptadas que se promedian
    double phi_max;
    double inflacion_fallo;      // factor de P de psi_dot y psi_ddot al rechazar o faltar una medicion
    double tope_inflacion;       // tope de esa inflacion, en multiplos de la varianza del prior de barrido
    int confirmar_maniobra;      // rechazos seguidos para pasar al modo de barrido (SELECCION_NIS)
    double nis_maniobra;         // NIS por sobre el cual un solo rechazo ya es maniobra (0: no)
    int histeresis_modo;         // vaciar la ventana de NIS al entrar al modo de barrido
    int max_fallos;              // fallos seguidos antes de reiniciar psi (se conserva rho)
    double sigma_perdido;        // sigma de psi (rad) por sobre la cual el blanco se da por perdido (0: no)
} ConfigEKF;

// Una medicion: instante, pixel, su sigma, la pose en ese instante y, si aplica, el tamano.
typedef struct {
    double t;      // instante de medicion t_m (s)
    double z[2];   // px
    double sigma;  // px
    double q[2];   // q_hat(t_m) (rad)
    double tamano; // px (NaN si no hay)
} MedicionEKF;

typedef struct {
    ConfigEKF cfg;
    Geometria geo;
    double xi[EKF_N], P[EKF_N * EKF_N];
    double t;            // instante al que corresponde el estado (s)
    int inicializado, hay_pendiente, fallos;
    double pend_psi[2], pend_t, pend_P[4]; // primera medicion del arranque
    double phi_k;
    int modo;
    // SELECCION_IMM: estado, covarianza y probabilidad de cada modo; xi y P son su combinacion.
    double xm[2][EKF_N], Pm[2][EKF_N * EKF_N], mu[2];
    double hist_nis[EKF_VENTANA_MAX];
    int hist_n, hist_pos;
} EKFPieza;

void ekf_iniciar(EKFPieza *f, const ConfigEKF *cfg, const Geometria *geo);

// Lleva el estado y la covarianza hasta el instante t (t >= f->t).
void ekf_predecir(EKFPieza *f, double t);

// Corrige con una medicion (antes predice hasta m->t). Devuelve 1 si se acepto (las dos
// primeras del arranque tambien cuentan como aceptadas, con NIS = NaN). nis_out puede ser NULL.
int ekf_corregir(EKFPieza *f, const MedicionEKF *m, double *nis_out);

// Sin deteccion en el instante t: predice y cuenta el fallo (infla P; reinicia psi tras
// max_fallos, conservando rho).
void ekf_sin_medicion(EKFPieza *f, double t);

// Estado predicho en t sin modificar el filtro (para el controlador): xi(t) y P(t).
void ekf_prediccion(const EKFPieza *f, double t, double xi[EKF_N], double P[EKF_N * EKF_N]);

// Prediccion del EKF en la imagen en el instante t con la pose q: s_hat = h(q, psi_hat(t), rho_hat) y
// S = H P H^T + sigma^2 I, con H = [H_psi 0 0 H_rho] (geo_jacobianos; xi = [psi, psi_dot, psi_ddot, rho] con psi en 0-1 y rho en 6). No modifica el
// filtro. Devuelve 0 si el EKF no esta inicializado o si el blanco queda detras de la camara.
int ekf_puerta_imagen(const EKFPieza *f, const Geometria *g, const double q[2], double t, double sigma,
                      double s[2], double S[4]);

// Lleva la elevacion de un estado predicho al rango [mu_min, mu_max] y anula mu_dot fuera de el:
// mas alla la referencia es inalcanzable y, pasada la vertical, la mirada g(psi) cambia de rama.
void ekf_saturar_elevacion(double xi[EKF_N], double mu_min, double mu_max);

// Encoge las componentes k y k+1 de xi por g (los canales de confianza del controlador, sobre
// una copia del filtro). Con IMM tambien las de cada modo, de donde sale ekf_prediccion.
void ekf_encoger(EKFPieza *f, int k, double g);

// Matrices de transicion y ruido de proceso del filtro para un intervalo dt (7x7).
void ekf_FQ(const EKFPieza *f, double dt, double F[EKF_N * EKF_N], double Q[EKF_N * EKF_N]);

void ekf_reiniciar(EKFPieza *f);

#ifdef __cplusplus
}
#endif

#endif /* EKF_PIEZA_H */

// Datos de un cuadro del modo vivo para los paneles del dashboard. El hilo del lazo llena un
// DatosCuadro y lo publica; el hilo del dashboard lo serializa con datos_cuadro_json y lo
// envia por SSE. Los campos de cada bloque solo valen desde el nivel indicado.
#ifndef DATOS_CUADRO_H
#define DATOS_CUADRO_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DC_HORIZONTE 11  // NMPC_HORIZONTE + 1
#define DC_M_FOV 8       // = M_FOV del NMPC

typedef struct {
    long long k;
    double t_captura_ms;
    int nivel, bloqueado;
    char fuente[12], perfil[8], estado_perfil[24];
    // percepcion (nivel >= 1)
    int hay_z;
    double z[2], sigma, confianza, ventana[4];  // ventana: x, y, ancho, alto (px); confianza 0..1
    // estimacion (nivel >= 2)
    int ekf_activo, aceptada, dim_nis;
    double s_hat[2], S[4], nis, nis_ref, mu[2], rho;
    double chi2_puerta;  // umbral chi^2 de la puerta (EKF_CHI2_2 o _3 segun dim_nis); nis_ref es otra cosa
    double phi_k;        // factor de la Q adaptativa
    int fallos_ekf, modo_ekf;  // fallos seguidos y modo de Singer (0 reposo, 1 barrido)
    // control (nivel >= 3)
    int estado_qp, qp_iter, n_horizonte;
    double holgura, ms_rti, ms_qp, delta_fb_ms, ta_menos_ts_ms;
    double horizonte[DC_HORIZONTE][2], fov_r, fov_b[DC_M_FOV], s_obj[2], centro[2];  // centro = (cu, cv)
    // comunicacion (nivel 4; en sombra solo bytes_plan)
    int bytes_plan, limitador, frenando, banderas_esp32;  // banderas: OR de las TEL_* del cuadro
    // ESP32 en los niveles 0 a 3 (calibracion): telemetria reciente (< 0,5 s), banderas CALIB_*
    // (-1 sin estado), pose medida y limites (rad)
    int esp_ok, esp_calib;
    double esp_q[2], esp_qmin[2], esp_qmax[2];
    int esp_sentido[2];      // SENTIDO_* de la ultima prueba de sentido de cada eje
    double esp_dq[2];        // lo que midio el encoder en esa prueba (grados)
    // calibracion de la camara (nivel 0): fotos validas, si la ultima tenia tablero (-1: ninguna),
    // si hay resultado, su RMS (px), fu fv cu cv (px) y k1 k2
    int cam_fotos, cam_ultima, cam_ok;
    double cam_rms, cam_K[4], cam_k[2];
    double edad_buffer_ms, ge_p, margen;
    // costos del hilo del lazo
    double ms_percepcion, ms_ctl, us_publicar;
} DatosCuadro;

// Todo en NaN (reales) o 0 (enteros), cadenas vacias.
void datos_cuadro_vaciar(DatosCuadro *d);

// JSON en una linea, con los nombres de los campos como claves; NaN e Inf salen como null y
// el horizonte lleva solo n_horizonte pares. Devuelve el largo, o -1 si no cabe en cap
// (incluido el terminador).
int datos_cuadro_json(const DatosCuadro *d, char *dst, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* DATOS_CUADRO_H */

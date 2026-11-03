// Datos de un cuadro para el dashboard (ver datos_cuadro.h): todo lo que se sabe de un cuadro
// de video (deteccion, estimacion del EKF, plan del NMPC, comunicacion, tiempos) se guarda en
// la estructura DatosCuadro y aqui se convierte a texto JSON para mandarlo al navegador.
// JSON es el formato de texto {"clave": valor, ...} que JavaScript entiende directamente.
#include "datos_cuadro.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Escritor con largo acumulado: tras un desborde ya no escribe y deja ok = 0.
typedef struct {
    char *dst;
    size_t cap, n;
    int ok;
} Esc;

// Agrega len bytes al final del texto. Si no caben, marca el escritor como fallido y, desde
// ahi, las demas escrituras no hacen nada (asi no hay que revisar cada llamada).
static void esc_crudo(Esc *e, const char *s, size_t len) {
    if (!e->ok) return;
    if (e->n + len + 1 > e->cap) {
        e->ok = 0;
        return;
    }
    memcpy(e->dst + e->n, s, len);
    e->n += len;
    e->dst[e->n] = '\0';
}

// Variantes del escritor: texto, entero y real. Un real que no es finito (NaN = "no hay
// dato") se escribe como null, porque JSON no tiene NaN.
static void esc_txt(Esc *e, const char *s) { esc_crudo(e, s, strlen(s)); }

static void esc_ent(Esc *e, long long v) {
    char b[32];
    int n = snprintf(b, sizeof b, "%lld", v);
    esc_crudo(e, b, (size_t)n);
}

static void esc_real(Esc *e, double v) {
    if (!isfinite(v)) {
        esc_txt(e, "null");
        return;
    }
    char b[40];
    int n = snprintf(b, sizeof b, "%.6g", v);
    esc_crudo(e, b, (size_t)n);
}

// Instante absoluto en ms desde el arranque: %.3f conserva el microsegundo con valores de
// hasta 1e15; mas alla (no es un instante real) se cae a %.6g para acotar el largo.
static void esc_tiempo(Esc *e, double v) {
    if (!isfinite(v)) {
        esc_txt(e, "null");
        return;
    }
    if (fabs(v) >= 1e15) {
        esc_real(e, v);
        return;
    }
    char b[40];
    int n = snprintf(b, sizeof b, "%.3f", v);
    esc_crudo(e, b, (size_t)n);
}

// Cadena JSON con comillas; escapa comillas, barra y caracteres de control.
static void esc_cad(Esc *e, const char *s, size_t max) {
    esc_crudo(e, "\"", 1);
    for (size_t i = 0; i < max && s[i]; i++) {
        unsigned char c = (unsigned char)s[i];
        char b[8];
        if (c == '"' || c == '\\') {
            b[0] = '\\';
            b[1] = (char)c;
            esc_crudo(e, b, 2);
        } else if (c == '\n') {
            esc_txt(e, "\\n");
        } else if (c == '\r') {
            esc_txt(e, "\\r");
        } else if (c == '\t') {
            esc_txt(e, "\\t");
        } else if (c < 0x20) {
            int n = snprintf(b, sizeof b, "\\u%04x", c);
            esc_crudo(e, b, (size_t)n);
        } else {
            esc_crudo(e, (const char *)&c, 1);
        }
    }
    esc_crudo(e, "\"", 1);
}

// Escribe ,"clave": antes de un valor. Las funciones k_* que siguen combinan la clave con su
// valor: k_ent (entero), k_real (real), k_cad (texto) y k_arr (arreglo de reales).
static void clave(Esc *e, const char *k) {
    esc_txt(e, ",\"");
    esc_txt(e, k);
    esc_txt(e, "\":");
}

static void k_ent(Esc *e, const char *k, long long v) {
    clave(e, k);
    esc_ent(e, v);
}

static void k_real(Esc *e, const char *k, double v) {
    clave(e, k);
    esc_real(e, v);
}

static void k_cad(Esc *e, const char *k, const char *s, size_t max) {
    clave(e, k);
    esc_cad(e, s, max);
}

static void k_arr(Esc *e, const char *k, const double *v, int n) {
    clave(e, k);
    esc_txt(e, "[");
    for (int i = 0; i < n; i++) {
        if (i) esc_txt(e, ",");
        esc_real(e, v[i]);
    }
    esc_txt(e, "]");
}

// Deja un DatosCuadro "vacio": enteros en 0 y reales en NaN. NaN significa "este cuadro no
// tiene ese dato" (por ejemplo, sin deteccion no hay z), y el navegador lo recibe como null.
// El arreglo "reales" guarda las direcciones de los campos reales para ponerlos todos en NaN
// con un solo ciclo.
void datos_cuadro_vaciar(DatosCuadro *d) {
    memset(d, 0, sizeof *d);
    double *reales[] = {&d->t_captura_ms, &d->sigma, &d->confianza, &d->nis, &d->nis_ref, &d->chi2_puerta, &d->phi_k,
                        &d->rho, &d->holgura, &d->ms_rti, &d->ms_qp, &d->delta_fb_ms,
                        &d->ta_menos_ts_ms, &d->fov_r, &d->edad_buffer_ms, &d->ge_p,
                        &d->margen, &d->ms_percepcion, &d->ms_ctl, &d->us_publicar};
    for (size_t i = 0; i < sizeof reales / sizeof reales[0]; i++) *reales[i] = NAN;
    for (int i = 0; i < 2; i++) d->z[i] = d->s_hat[i] = d->mu[i] = d->s_obj[i] = d->centro[i] = NAN;
    for (int i = 0; i < 2; i++) d->esp_q[i] = d->esp_qmin[i] = d->esp_qmax[i] = NAN;
    d->esp_calib = d->cam_ultima = -1;
    d->cam_rms = NAN;
    for (int i = 0; i < 2; i++) d->esp_dq[i] = d->cam_k[i] = NAN;
    for (int i = 0; i < 4; i++) d->cam_K[i] = NAN;
    for (int i = 0; i < 4; i++) d->ventana[i] = d->S[i] = NAN;
    for (int i = 0; i < DC_M_FOV; i++) d->fov_b[i] = NAN;
    for (int i = 0; i < DC_HORIZONTE; i++) d->horizonte[i][0] = d->horizonte[i][1] = NAN;
}

// Convierte el cuadro completo a JSON. Devuelve el largo escrito, o -1 si no cupo en dst.
// El orden de las claves no importa para el navegador, pero se agrupan por tema.
int datos_cuadro_json(const DatosCuadro *d, char *dst, size_t cap) {
    Esc e = {dst, cap, 0, cap > 0};
    esc_txt(&e, "{\"k\":");
    esc_ent(&e, d->k);
    clave(&e, "t_captura_ms");
    esc_tiempo(&e, d->t_captura_ms);
    k_ent(&e, "nivel", d->nivel);
    k_ent(&e, "bloqueado", d->bloqueado);
    k_cad(&e, "fuente", d->fuente, sizeof d->fuente);
    k_cad(&e, "perfil", d->perfil, sizeof d->perfil);
    k_cad(&e, "estado_perfil", d->estado_perfil, sizeof d->estado_perfil);

    k_ent(&e, "hay_z", d->hay_z);
    k_arr(&e, "z", d->z, 2);
    k_real(&e, "sigma", d->sigma);
    k_real(&e, "confianza", d->confianza);
    k_arr(&e, "ventana", d->ventana, 4);

    k_ent(&e, "ekf_activo", d->ekf_activo);
    k_ent(&e, "aceptada", d->aceptada);
    k_ent(&e, "dim_nis", d->dim_nis);
    k_arr(&e, "s_hat", d->s_hat, 2);
    k_arr(&e, "S", d->S, 4);
    k_real(&e, "nis", d->nis);
    k_real(&e, "nis_ref", d->nis_ref);
    k_real(&e, "chi2_puerta", d->chi2_puerta);
    k_real(&e, "phi_k", d->phi_k);
    k_ent(&e, "fallos_ekf", d->fallos_ekf);
    k_ent(&e, "modo_ekf", d->modo_ekf);
    k_arr(&e, "mu", d->mu, 2);
    k_real(&e, "rho", d->rho);

    k_ent(&e, "estado_qp", d->estado_qp);
    k_ent(&e, "qp_iter", d->qp_iter);
    k_ent(&e, "n_horizonte", d->n_horizonte);
    k_real(&e, "holgura", d->holgura);
    k_real(&e, "ms_rti", d->ms_rti);
    k_real(&e, "ms_qp", d->ms_qp);
    k_real(&e, "delta_fb_ms", d->delta_fb_ms);
    k_real(&e, "ta_menos_ts_ms", d->ta_menos_ts_ms);
    clave(&e, "horizonte");
    esc_txt(&e, "[");
    int nh = d->n_horizonte < 0 ? 0 : d->n_horizonte > DC_HORIZONTE ? DC_HORIZONTE : d->n_horizonte;
    for (int i = 0; i < nh; i++) {
        if (i) esc_txt(&e, ",");
        esc_txt(&e, "[");
        esc_real(&e, d->horizonte[i][0]);
        esc_txt(&e, ",");
        esc_real(&e, d->horizonte[i][1]);
        esc_txt(&e, "]");
    }
    esc_txt(&e, "]");
    k_real(&e, "fov_r", d->fov_r);
    k_arr(&e, "fov_b", d->fov_b, DC_M_FOV);
    k_arr(&e, "s_obj", d->s_obj, 2);
    k_arr(&e, "centro", d->centro, 2);

    k_ent(&e, "bytes_plan", d->bytes_plan);
    k_ent(&e, "limitador", d->limitador);
    k_ent(&e, "frenando", d->frenando);
    k_ent(&e, "banderas_esp32", d->banderas_esp32);
    k_ent(&e, "esp_ok", d->esp_ok);
    k_ent(&e, "esp_calib", d->esp_calib);
    k_arr(&e, "esp_q", d->esp_q, 2);
    k_arr(&e, "esp_qmin", d->esp_qmin, 2);
    k_arr(&e, "esp_qmax", d->esp_qmax, 2);
    k_ent(&e, "esp_sentido0", d->esp_sentido[0]);
    k_ent(&e, "esp_sentido1", d->esp_sentido[1]);
    k_arr(&e, "esp_dq", d->esp_dq, 2);
    k_ent(&e, "cam_fotos", d->cam_fotos);
    k_ent(&e, "cam_ultima", d->cam_ultima);
    k_ent(&e, "cam_ok", d->cam_ok);
    k_real(&e, "cam_rms", d->cam_rms);
    k_arr(&e, "cam_K", d->cam_K, 4);
    k_arr(&e, "cam_k", d->cam_k, 2);
    k_real(&e, "edad_buffer_ms", d->edad_buffer_ms);
    k_real(&e, "ge_p", d->ge_p);
    k_real(&e, "margen", d->margen);

    k_real(&e, "ms_percepcion", d->ms_percepcion);
    k_real(&e, "ms_ctl", d->ms_ctl);
    k_real(&e, "us_publicar", d->us_publicar);
    esc_txt(&e, "}");
    if (!e.ok && cap > 0) dst[0] = '\0';
    return e.ok ? (int)e.n : -1;
    return e.ok ? (int)e.n : -1;
}

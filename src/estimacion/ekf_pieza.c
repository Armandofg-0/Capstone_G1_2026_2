// En palabras simples: el EKF (filtro de Kalman extendido) es el "estimador" del blanco. Con
// cada deteccion de la camara corrige su idea de hacia donde esta el blanco y como se mueve, y
// entre detecciones la extrapola con un modelo de movimiento (Singer). Tambien sabe cuanto
// confia en su estimacion (la matriz P) y rechaza detecciones que no calzan (la "puerta"). La
// direccion se guarda vista desde la base de la torreta, asi que no cambia cuando la torreta
// gira; el giro solo entra al comparar con la imagen.
// EKF en la pieza (ver ekf_pieza.h).
#include "ekf_pieza.h"

#include <math.h>
#include <string.h>

#include "singer_gen.h"

#define N EKF_N
#define IDX(r, c) ((r) * N + (c))

// Producto C = A B con A (n x m) y B (m x p), matrices planas por filas.
static void mult(const double *A, const double *B, double *C, int n, int m, int p)
{
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < p; ++j) {
            double s = 0.0;
            for (int k = 0; k < m; ++k)
                s += A[i * m + k] * B[k * p + j];
            C[i * p + j] = s;
        }
}
//---------------------------------------------------------------------------
// Inversa de una simetrica definida positiva de 2x2 o 3x3 por cofactores. Devuelve 0 si es
// singular.
static int invertir(const double *S, int m, double *Si)
{
    if (m == 2) {
        const double det = S[0] * S[3] - S[1] * S[2];
        if (!(det > 0.0))
            return 0;
        Si[0] = S[3] / det;
        Si[1] = -S[1] / det;
        Si[2] = -S[2] / det;
        Si[3] = S[0] / det;
        return 1;
    }
    const double a = S[0], b = S[1], c = S[2], d = S[3], e = S[4], f = S[5], g = S[6], h = S[7], k = S[8];
    const double A = e * k - f * h, B = -(d * k - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    if (!(det > 0.0))
        return 0;
    Si[0] = A / det;
    Si[1] = -(b * k - c * h) / det;
    Si[2] = (b * f - c * e) / det;
    Si[3] = B / det;
    Si[4] = (a * k - c * g) / det;
    Si[5] = -(a * f - c * d) / det;
    Si[6] = C / det;
    Si[7] = -(a * h - b * g) / det;
    Si[8] = (a * e - b * d) / det;
    return 1;
}
//---------------------------------------------------------------------------
// Determinante de una simetrica de 2x2 o 3x3.
static double determinante(const double *S, int m)
{
    if (m == 2)
        return S[0] * S[3] - S[1] * S[2];
    return S[0] * (S[4] * S[8] - S[5] * S[7]) - S[1] * (S[3] * S[8] - S[5] * S[6])
           + S[2] * (S[3] * S[7] - S[4] * S[6]);
}
//---------------------------------------------------------------------------
// Fuerza que P sea simetrica (promedia P con su transpuesta): los redondeos de la aritmetica la
// van desarmando de a poco y una P no simetrica puede volver inestable al filtro.
static void simetrizar(double *P)
{
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
            P[IDX(i, j)] = P[IDX(j, i)] = 0.5 * (P[IDX(i, j)] + P[IDX(j, i)]);
}
//---------------------------------------------------------------------------
// Modo de Singer con el que predice el filtro: el 0 con SELECCION_FIJO; si no, el activo (con
// IMM, el mas probable).
static int modo_activo(const EKFPieza *f)
{
    return f->cfg.seleccion == SELECCION_FIJO ? 0 : f->modo;
}
//---------------------------------------------------------------------------
// F y Q de Singer del modo m para el intervalo dt (7x7).
static void fq_modo(const EKFPieza *f, int m, double dt, double F[N * N], double Q[N * N])
{
    double F1[9], Q1[9];
    singer_FQ(dt, f->cfg.tau_m[m], f->cfg.sigma_m[m], F1, Q1);
    memset(F, 0, N * N * sizeof(double));
    memset(Q, 0, N * N * sizeof(double));
    // F1 (x) I2: la variable a (0 = psi, 1 = psi_dot, 2 = psi_ddot) del eje j esta en la posicion 2a + j.
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            for (int j = 0; j < 2; ++j) {
                F[IDX(2 * a + j, 2 * b + j)] = F1[3 * a + b];
                Q[IDX(2 * a + j, 2 * b + j)] = f->phi_k * Q1[3 * a + b];
            }
    F[IDX(6, 6)] = 1.0;
    Q[IDX(6, 6)] = f->cfg.tratamiento_rho == RHO_ESTIMADO ? f->cfg.q_rho * dt : 0.0;
}
//---------------------------------------------------------------------------
// Matrices de transicion F y de ruido de proceso Q del modo activo para un intervalo dt.
void ekf_FQ(const EKFPieza *f, double dt, double F[N * N], double Q[N * N])
{
    fq_modo(f, modo_activo(f), dt, F, Q);
}
//---------------------------------------------------------------------------
// x = F x, P = F P F^T + Q con el modo m para el intervalo dt.
static void propagar(const EKFPieza *f, int m, double dt, double xi[N], double P[N * N])
{
    double F[N * N], Q[N * N], x[N], FP[N * N], Ft[N * N];
    fq_modo(f, m, dt, F, Q);
    mult(F, xi, x, N, N, 1);
    memcpy(xi, x, sizeof(x));
    mult(F, P, FP, N, N, N);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            Ft[IDX(i, j)] = F[IDX(j, i)];
    mult(FP, Ft, P, N, N, N);
    for (int i = 0; i < N * N; ++i)
        P[i] += Q[i];
}
//---------------------------------------------------------------------------
// IMM (Blom y Bar-Shalom). Las permanencias medias T_j definen una cadena de Markov continua
// con generador [-1/T0, 1/T0; 1/T1, -1/T1], cuya transicion para un intervalo Delta t es exacta:
// pi_ij = p_j + (delta_ij - p_j) e^(-lambdaDelta t), con lambda = 1/T0 + 1/T1 y la estacionaria p_j = T_j / (T0 + T1).
// Para Delta t chico, pi_jj ~ 1 - Delta t/T_j; y con Delta t variable, dos tramos equivalen a uno.
static void transicion(const ConfigEKF *c, double dt, double pi[2][2])
{
    const double T = c->imm_T[0] + c->imm_T[1];
    const double e = exp(-(1.0 / c->imm_T[0] + 1.0 / c->imm_T[1]) * dt);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const double p = c->imm_T[j] / T;
            pi[i][j] = p + ((i == j) - p) * e;
        }
}
//---------------------------------------------------------------------------
// Probabilidades de los dos modos en regimen permanente, segun cuanto se queda en cada uno.
static void mu_estacionaria(EKFPieza *f)
{
    const double T = f->cfg.imm_T[0] + f->cfg.imm_T[1];
    f->mu[0] = f->cfg.imm_T[0] / T;
    f->mu[1] = f->cfg.imm_T[1] / T;
}
//---------------------------------------------------------------------------
// Mezcla por momentos de los dos modos con pesos w: x = Sigma w_j x_j y
// P = Sigma w_j [P_j + (x_j - x)(x_j - x)^T].
static void mezclar(const double xm[2][N], const double Pm[2][N * N], const double w[2], double x[N],
                    double P[N * N])
{
    for (int i = 0; i < N; ++i)
        x[i] = w[0] * xm[0][i] + w[1] * xm[1][i];
    for (int i = 0; i < N; ++i)
        for (int k = 0; k < N; ++k) {
            double s = 0.0;
            for (int j = 0; j < 2; ++j)
                s += w[j] * (Pm[j][IDX(i, k)] + (xm[j][i] - x[i]) * (xm[j][k] - x[k]));
            P[IDX(i, k)] = s;
        }
    simetrizar(P);
}
//---------------------------------------------------------------------------
// Lo que ven los consumidores: la combinacion de los modos, y el mas probable como modo.
static void combinar(EKFPieza *f)
{
    mezclar((const double(*)[N])f->xm, (const double(*)[N * N])f->Pm, f->mu, f->xi, f->P);
    f->modo = f->mu[1] > f->mu[0];
}
//---------------------------------------------------------------------------
// Interaccion: cada modo parte de la mezcla de los dos con la probabilidad de venir de cada
// uno, mu_{i|j} = pi_ij mu_i / c_j con c_j = Sigma_i pi_ij mu_i, y mu pasa a ser la predicha, c. Si
// c_j es nulo (mu_j nula y Delta t = 0), el modo se queda con lo suyo.
static void interactuar(EKFPieza *f, double dt)
{
    double pi[2][2], c[2], x0[2][N], P0[2][N * N];
    transicion(&f->cfg, dt, pi);
    for (int j = 0; j < 2; ++j)
        c[j] = pi[0][j] * f->mu[0] + pi[1][j] * f->mu[1];
    for (int j = 0; j < 2; ++j) {
        if (!(c[j] > 0.0)) {
            memcpy(x0[j], f->xm[j], sizeof(x0[j]));
            memcpy(P0[j], f->Pm[j], sizeof(P0[j]));
            continue;
        }
        const double w[2] = {pi[0][j] * f->mu[0] / c[j], pi[1][j] * f->mu[1] / c[j]};
        mezclar((const double(*)[N])f->xm, (const double(*)[N * N])f->Pm, w, x0[j], P0[j]);
    }
    memcpy(f->xm, x0, sizeof(x0));
    memcpy(f->Pm, P0, sizeof(P0));
    const double s = c[0] + c[1];
    f->mu[0] = c[0] / s;
    f->mu[1] = c[1] / s;
}
//---------------------------------------------------------------------------
// Paso del IMM sin medicion: interaccion, prediccion de cada modo con su Singer, combinacion.
static void predecir_imm(EKFPieza *f, double dt)
{
    interactuar(f, dt);
    for (int j = 0; j < 2; ++j)
        propagar(f, j, dt, f->xm[j], f->Pm[j]);
    combinar(f);
}
//---------------------------------------------------------------------------
// Tras el arranque de dos puntos, los dos modos parten del mismo estado, cada uno con la
// varianza estacionaria de psi_ddot de su Singer, y mu en la estacionaria.
static void iniciar_modos(EKFPieza *f)
{
    for (int j = 0; j < 2; ++j) {
        memcpy(f->xm[j], f->xi, sizeof(f->xi));
        memcpy(f->Pm[j], f->P, sizeof(f->P));
        for (int a = 0; a < 2; ++a)
            f->Pm[j][IDX(4 + a, 4 + a)] = f->cfg.sigma_m[j] * f->cfg.sigma_m[j];
    }
    mu_estacionaria(f);
    combinar(f);
}
//---------------------------------------------------------------------------
// Un modo como filtro propio, para reutilizar evaluar y corregir_con.
static void cargar_modo(const EKFPieza *f, int j, EKFPieza *g)
{
    *g = *f;
    memcpy(g->xi, f->xm[j], sizeof(g->xi));
    memcpy(g->P, f->Pm[j], sizeof(g->P));
    g->modo = j;
}
// Lo inverso: devuelve al modo j el estado y la covarianza corregidos.
static void guardar_modo(EKFPieza *f, int j, const EKFPieza *g)
{
    memcpy(f->xm[j], g->xi, sizeof(g->xi));
    memcpy(f->Pm[j], g->P, sizeof(g->P));
}
//---------------------------------------------------------------------------
// Deja el filtro listo pero sin inicializar: arranca con las dos primeras detecciones.
void ekf_iniciar(EKFPieza *f, const ConfigEKF *cfg, const Geometria *geo)
{
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    if (f->cfg.ventana > EKF_VENTANA_MAX)
        f->cfg.ventana = EKF_VENTANA_MAX;
    if (f->cfg.ventana < 1)
        f->cfg.ventana = 1;
    f->geo = *geo;
    f->xi[6] = cfg->rho0;
    f->P[IDX(6, 6)] = cfg->sigma_rho0 * cfg->sigma_rho0;
    f->phi_k = 1.0;
    if (cfg->seleccion == SELECCION_IMM)
        mu_estacionaria(f);
}
//---------------------------------------------------------------------------
// Olvida el blanco pero conserva rho y su varianza (informe, "Sin deteccion").
void ekf_reiniciar(EKFPieza *f)
{
    const double rho = f->xi[6], prr = f->P[IDX(6, 6)];
    memset(f->xi, 0, sizeof(f->xi));
    memset(f->P, 0, sizeof(f->P));
    f->xi[6] = rho;
    f->P[IDX(6, 6)] = prr;
    f->inicializado = f->hay_pendiente = f->fallos = 0;
    f->phi_k = 1.0;
    f->modo = 0;
    f->hist_n = f->hist_pos = 0;
    if (f->cfg.seleccion == SELECCION_IMM)
        mu_estacionaria(f);
}
//---------------------------------------------------------------------------
// Avanza el estado y su covarianza hasta el instante t con el modelo de Singer.
void ekf_predecir(EKFPieza *f, double t)
{
    if (f->inicializado && t > f->t) {
        if (f->cfg.seleccion == SELECCION_IMM)
            predecir_imm(f, t - f->t);
        else
            propagar(f, modo_activo(f), t - f->t, f->xi, f->P);
    }
    if (t > f->t || !f->inicializado)
        f->t = t;
}
//---------------------------------------------------------------------------
// Con IMM, la prediccion de cada modo desde su estado combinada con mu(t) = mu pi(t - t_f). En el
// horizonte del controlador no se interactua: es una aproximacion (los modos no se mezclan
// entre los pasos de prediccion).
void ekf_prediccion(const EKFPieza *f, double t, double xi[N], double P[N * N])
{
    if (f->cfg.seleccion == SELECCION_IMM && f->inicializado && t > f->t) {
        const double dt = t - f->t;
        double pi[2][2], x[2][N], Pp[2][N * N], w[2];
        transicion(&f->cfg, dt, pi);
        memcpy(x, f->xm, sizeof(x));
        memcpy(Pp, f->Pm, sizeof(Pp));
        for (int j = 0; j < 2; ++j) {
            propagar(f, j, dt, x[j], Pp[j]);
            w[j] = pi[0][j] * f->mu[0] + pi[1][j] * f->mu[1];
        }
        mezclar((const double(*)[N])x, (const double(*)[N * N])Pp, w, xi, P);
        return;
    }
    memcpy(xi, f->xi, sizeof(f->xi));
    memcpy(P, f->P, sizeof(f->P));
    if (f->inicializado && t > f->t)
        propagar(f, modo_activo(f), t - f->t, xi, P);
}
//---------------------------------------------------------------------------
// Proyecta la prediccion del EKF a la imagen: pixel esperado s y su covarianza S (incluida la
// incertidumbre sigma de la deteccion). La percepcion la usa para buscar el blanco solo en la
// zona probable (la "pista").
int ekf_puerta_imagen(const EKFPieza *f, const Geometria *g, const double q[2], double t, double sigma,
                      double s[2], double S[4])
{
    if (!f->inicializado)
        return 0;
    double xi[N], P[N * N];
    ekf_prediccion(f, t, xi, P);
    if (!(geo_az(q, xi) > 0.0))
        return 0;
    double Hq[4], Hpsi[4], Hrho[2];
    geo_h(g, q, xi, xi[6], s);
    geo_jacobianos(g, q, xi, xi[6], Hq, Hpsi, Hrho);
    // H (2x7): psi en las columnas 0-1, rho en la 6; las derivadas no entran a h.
    double H[2 * N] = {0};
    for (int i = 0; i < 2; ++i) {
        H[i * N + 0] = Hpsi[i * 2 + 0];
        H[i * N + 1] = Hpsi[i * 2 + 1];
        H[i * N + 6] = Hrho[i];
    }
    double HP[2 * N], HT[N * 2];
    mult(H, P, HP, 2, N, N);
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < N; ++k)
            HT[k * 2 + i] = H[i * N + k];
    mult(HP, HT, S, 2, N, 2);
    S[0] += sigma * sigma;
    S[3] += sigma * sigma;
    return 1;
}
//---------------------------------------------------------------------------
// Limita la elevacion predicha (mu) al rango que alcanza el tilt.
void ekf_saturar_elevacion(double xi[N], double mu_min, double mu_max)
{
    if (xi[1] > mu_max || xi[1] < mu_min) {
        xi[1] = xi[1] > mu_max ? mu_max : mu_min;
        xi[3] = 0.0;
    }
}
//---------------------------------------------------------------------------
// Con IMM se encoge cada modo: la combinacion es lineal en ellos, asi que xi queda encogida
// igual, y ekf_prediccion (que sale de los modos) ve el cambio. La covarianza no se toca.
void ekf_encoger(EKFPieza *f, int k, double g)
{
    f->xi[k] *= g;
    f->xi[k + 1] *= g;
    if (f->cfg.seleccion == SELECCION_IMM)
        for (int j = 0; j < 2; ++j) {
            f->xm[j][k] *= g;
            f->xm[j][k + 1] *= g;
        }
}
//---------------------------------------------------------------------------
// Varianza de psi que implica una medicion: P_psi = H_psi^-1 R H_psi^-^T (arranque).
static void var_psi(const EKFPieza *f, const MedicionEKF *m, const double psi[2], double Ppsi[4])
{
    double Hq[4], Hp[4], Hr[2], Hi[4], R[4], T[4], Hit[4];
    geo_jacobianos(&f->geo, m->q, psi, f->xi[6], Hq, Hp, Hr);
    const double det = Hp[0] * Hp[3] - Hp[1] * Hp[2];
    Hi[0] = Hp[3] / det;
    Hi[1] = -Hp[1] / det;
    Hi[2] = -Hp[2] / det;
    Hi[3] = Hp[0] / det;
    const double s2 = m->sigma * m->sigma, q2 = f->cfg.sigma_q * f->cfg.sigma_q;
    R[0] = s2 + q2 * (Hq[0] * Hq[0] + Hq[1] * Hq[1]);
    R[1] = R[2] = q2 * (Hq[0] * Hq[2] + Hq[1] * Hq[3]);
    R[3] = s2 + q2 * (Hq[2] * Hq[2] + Hq[3] * Hq[3]);
    mult(Hi, R, T, 2, 2, 2);
    Hit[0] = Hi[0];
    Hit[1] = Hi[2];
    Hit[2] = Hi[1];
    Hit[3] = Hi[3];
    mult(T, Hit, Ppsi, 2, 2, 2);
}
//---------------------------------------------------------------------------
// Arranque de dos puntos. Devuelve 0 si la medicion no se puede invertir.
static int arrancar(EKFPieza *f, const MedicionEKF *m)
{
    // Con el tamano aparente, la primera medicion tambien fija rho: rho = (z_w - b_w) a_z / k_w.
    double psi[2];
    if (!geo_inversa(&f->geo, m->q, m->z, f->xi[6], psi))
        return 0;
    if (f->cfg.usar_tamano && f->cfg.tratamiento_rho == RHO_ESTIMADO && isfinite(m->tamano) && !f->hay_pendiente) {
        const double rho = (m->tamano - f->cfg.bw) * geo_az(m->q, psi) / f->cfg.kw;
        if (rho > f->cfg.rho_min && rho < f->cfg.rho_max) {
            f->xi[6] = rho;
            const double s = f->cfg.sigma_w * geo_az(m->q, psi) / f->cfg.kw;
            f->P[IDX(6, 6)] = s * s;
            geo_inversa(&f->geo, m->q, m->z, f->xi[6], psi);
        }
    }
    double Ppsi[4];
    var_psi(f, m, psi, Ppsi);
    const double dt = m->t - f->pend_t;
    if (!f->hay_pendiente || !(dt > 0.0)) {
        f->hay_pendiente = 1;
        f->pend_psi[0] = psi[0];
        f->pend_psi[1] = psi[1];
        f->pend_t = m->t;
        memcpy(f->pend_P, Ppsi, sizeof(Ppsi));
        return 1;
    }
    const double rho = f->xi[6], prr = f->P[IDX(6, 6)];
    memset(f->xi, 0, sizeof(f->xi));
    memset(f->P, 0, sizeof(f->P));
    const double sm = f->cfg.sigma_m[0];
    for (int a = 0; a < 2; ++a) {
        f->xi[a] = psi[a];
        f->xi[2 + a] = (psi[a] - f->pend_psi[a]) / dt;
        for (int b = 0; b < 2; ++b) {
            f->P[IDX(a, b)] = Ppsi[2 * a + b];
            f->P[IDX(2 + a, 2 + b)] = (Ppsi[2 * a + b] + f->pend_P[2 * a + b]) / (dt * dt);
            f->P[IDX(a, 2 + b)] = f->P[IDX(2 + b, a)] = Ppsi[2 * a + b] / dt;
        }
        f->P[IDX(4 + a, 4 + a)] = sm * sm; // aceleracion estacionaria de Singer
    }
    f->xi[6] = rho;
    f->P[IDX(6, 6)] = prr;
    f->inicializado = 1;
    f->hay_pendiente = 0;
    f->fallos = 0;
    f->t = m->t;
    return 1;
}
//---------------------------------------------------------------------------
// Actualizacion de Joseph con una ganancia cualquiera K (N x m):
//   P = (I - K H) P (I - K H)^T + K R K^T.
// Vale tambien con la fila de rho forzada a cero (Schmidt-Kalman).
static void joseph(EKFPieza *f, const double *K, const double *H, const double *R, int m)
{
    double IKH[N * N], A[N * N], T[N * N], KR[N * 3];
    mult(K, H, IKH, N, m, N);
    for (int i = 0; i < N * N; ++i)
        IKH[i] = -IKH[i];
    for (int i = 0; i < N; ++i)
        IKH[IDX(i, i)] += 1.0;
    mult(IKH, f->P, A, N, N, N);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            T[IDX(i, j)] = IKH[IDX(j, i)];
    mult(A, T, f->P, N, N, N);
    mult(K, R, KR, N, m, m);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            double s = 0.0;
            for (int k = 0; k < m; ++k)
                s += KR[i * m + k] * K[j * m + k];
            f->P[IDX(i, j)] += s;
        }
    // Simetria exacta frente al redondeo.
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
            f->P[IDX(i, j)] = f->P[IDX(j, i)] = 0.5 * (f->P[IDX(i, j)] + f->P[IDX(j, i)]);
}
//---------------------------------------------------------------------------
// Prior de la maniobra: al pasar al modo de barrido, la velocidad y la aceleracion del
// blanco pueden ser las de ese modo (sigma_m tau_m y sigma_m), sin relacion con la estimacion de
// reposo. Se reinicia ese bloque de P a max(P, prior) y se anulan sus correlaciones con psi y
// con rho: la matriz queda definida positiva (diagonal por bloques).
static void prior_de_maniobra(EKFPieza *f)
{
    const double sv = f->cfg.sigma_m[1] * f->cfg.tau_m[1], sa = f->cfg.sigma_m[1];
    double D[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            D[i][j] = f->P[IDX(2 + i, 2 + j)];
    for (int i = 0; i < N; ++i)
        for (int j = 2; j < 6; ++j)
            if (i < 2 || i > 5)
                f->P[IDX(i, j)] = f->P[IDX(j, i)] = 0.0;
    for (int k = 0; k < 2; ++k) {
        if (D[k][k] < sv * sv)
            D[k][k] = sv * sv;
        if (D[2 + k][2 + k] < sa * sa)
            D[2 + k][2 + k] = sa * sa;
    }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            f->P[IDX(2 + i, 2 + j)] = (i == j) ? D[i][j] : 0.0;
}
//---------------------------------------------------------------------------
// Inflacion de P de psi_dot y psi_ddot por un fallo, con tope en el prior del modo mas agil (sigma_m tau_m y
// sigma_m del modo de barrido): compuesta sin tope (xinflacion_fallo por rechazo), llegaba a
// cientos de rad^2/s^2 y la siguiente aceptacion fijaba velocidades sin sentido (H23 de la
// fase 6). El bloque se multiplica por un unico factor f  en  [1, inflacion_fallo], lo que
// conserva P definida positiva; si ya esta en el tope, no cambia.
static void inflar_P(const ConfigEKF *c, double P[N * N])
{
    const double k = c->tope_inflacion;
    const double sv2 = k * pow(c->sigma_m[1] * c->tau_m[1], 2), sa2 = k * pow(c->sigma_m[1], 2);
    double fac = c->inflacion_fallo;
    for (int k = 0; k < 2; ++k) {
        const double pv = P[IDX(2 + k, 2 + k)], pa = P[IDX(4 + k, 4 + k)];
        if (pv > 0.0)
            fac = fmin(fac, sv2 / pv);
        if (pa > 0.0)
            fac = fmin(fac, sa2 / pa);
    }
    if (fac <= 1.0)
        return;
    for (int i = 2; i < 6; ++i)
        for (int j = 2; j < 6; ++j)
            P[IDX(i, j)] *= fac;
}
// Infla la covarianza del filtro (confia menos en lo que cree saber).
static void inflar(EKFPieza *f)
{
    inflar_P(&f->cfg, f->P);
}
//---------------------------------------------------------------------------
// Al entrar al modo de barrido, la ventana de NIS se vacia: la salida se decide con
// `ventana` mediciones evaluadas en el modo nuevo. Con la ventana llena de NIS de reposo,
// adaptar volvia al reposo en el mismo paso, el modelo de reposo extrapolaba la velocidad del
// prior y la medicion siguiente volvia a fallar (timbre de psi_dot, fase 6).
static void vaciar_ventana(EKFPieza *f)
{
    f->hist_n = f->hist_pos = 0;
}
//---------------------------------------------------------------------------
// Blanco perdido: la incertidumbre de psi supera la FOV. Con P asi, la puerta chi^2 acepta
// cualquier deteccion y cada aceptacion (que pone fallos = 0) fija velocidades sin sentido,
// asi que el contador de fallos nunca llega a reiniciar el filtro: mejor volver a arrancar
// con dos puntos.
static int perdido(const EKFPieza *f)
{
    const double s2 = f->cfg.sigma_perdido * f->cfg.sigma_perdido;
    return s2 > 0.0 && f->P[IDX(0, 0)] + f->P[IDX(1, 1)] > s2;
}
//---------------------------------------------------------------------------
// Rechazo: si es adaptativo, phi_k sube (evidencia de maniobra); con dos modos, tras
// confirmar_maniobra rechazos seguidos se pasa al de barrido y las derivadas toman su prior.
// Un rechazo aislado es lo que la puerta al 99 % da una vez cada cien mediciones sin maniobra:
// tomarlo como maniobra convertia un atipico en una velocidad espuria. Si no se cambia de
// modo, P de psi_dot y psi_ddot se infla, con tope (inflar). Con IMM se infla cada modo: mu ya es la
// predicha, porque un rechazo no dice nada sobre cual de los dos es.
static void rechazo(EKFPieza *f, double nis)
{
    const int confirmada = f->fallos + 1 >= f->cfg.confirmar_maniobra
                           || (f->cfg.nis_maniobra > 0.0 && nis > f->cfg.nis_maniobra);
    if (f->cfg.seleccion == SELECCION_IMM) {
        for (int j = 0; j < 2; ++j)
            inflar_P(&f->cfg, f->Pm[j]);
        combinar(f);
    } else if (f->cfg.seleccion == SELECCION_NIS && f->modo == 0 && confirmada) {
        f->modo = 1;
        prior_de_maniobra(f);
        if (f->cfg.histeresis_modo)
            vaciar_ventana(f);
    } else
        inflar(f);
    if (f->cfg.adaptativo && (f->phi_k *= 2.0) > f->cfg.phi_max)
        f->phi_k = f->cfg.phi_max;
    if (++f->fallos > f->cfg.max_fallos || perdido(f))
        ekf_reiniciar(f);
}
//---------------------------------------------------------------------------
// phi_k y el modo de Singer segun la media del NIS de las aceptadas (normalizado por la media
// truncada de la chi^2 de su dimension, para comparar mediciones de 2 y 3 componentes).
static void adaptar(EKFPieza *f, double nis_norm)
{
    const int v = f->cfg.ventana;
    f->hist_nis[f->hist_pos] = nis_norm;
    f->hist_pos = (f->hist_pos + 1) % v;
    if (f->hist_n < v)
        f->hist_n++;
    if (f->hist_n < v)
        return;
    double s = 0.0;
    for (int i = 0; i < v; ++i)
        s += f->hist_nis[i];
    const double prom = s / v; // 1 = consistente con la chi^2 truncada
    if (f->cfg.adaptativo) {
        if (prom > 3.0) {
            if ((f->phi_k *= 2.0) > f->cfg.phi_max)
                f->phi_k = f->cfg.phi_max;
        } else if (prom < 1.0 && (f->phi_k /= 1.5) < 1.0)
            f->phi_k = 1.0;
    }
    if (f->cfg.seleccion == SELECCION_NIS) {
        const double nis2 = prom * EKF_NIS_REF_2; // en unidades de NIS de 2 g.l.
        if (f->modo == 0 && nis2 > f->cfg.nis_subir)
            f->modo = 1;
        else if (f->modo == 1 && nis2 < f->cfg.nis_bajar)
            f->modo = 0;
    }
}
//---------------------------------------------------------------------------
// Pseudo-medicion de las cotas de rho (solo estimado): si rho_hat sale de [rho_min, rho_max], se mide
// rho = cota con varianza sigma^2_cota, lo que corrige media y covarianza de forma consistente.
static void cotas_rho(EKFPieza *f)
{
    const ConfigEKF *c = &f->cfg;
    if (c->tratamiento_rho != RHO_ESTIMADO)
        return;
    const double x = f->xi[6];
    if (x >= c->rho_min && x <= c->rho_max)
        return;
    const double cota = x < c->rho_min ? c->rho_min : c->rho_max;
    const double R = c->sigma_rho_cota * c->sigma_rho_cota;
    const double S = f->P[IDX(6, 6)] + R;
    double K[N], H[N] = {0, 0, 0, 0, 0, 0, 1};
    for (int i = 0; i < N; ++i)
        K[i] = f->P[IDX(i, 6)] / S;
    const double y = cota - x;
    for (int i = 0; i < N; ++i)
        f->xi[i] += K[i] * y;
    joseph(f, K, H, &R, 1);
}
//---------------------------------------------------------------------------
// Evaluacion de una medicion contra el estado predicho: H, residuo, R, S^-1 y NIS.
typedef struct {
    int nm, ok;
    double H[3 * N], y[3], R[9], PHt[N * 3], S[9], Si[9], nis;
} Evaluacion;

// Evalua una medicion sin aplicarla: predice lo que deberia verse (h), calcula el residuo y,
// el jacobiano H, la covarianza de la innovacion S y el NIS (que tan "sorprendente" es la
// medicion). Con eso se decide si pasa la puerta antes de corregir.
static void evaluar(const EKFPieza *f, const MedicionEKF *m, Evaluacion *e)
{
    const double *psi = f->xi, rho = f->xi[6];
    double s[2], Hq[4], Hp[4], Hr[2];
    geo_h(&f->geo, m->q, psi, rho, s);
    geo_jacobianos(&f->geo, m->q, psi, rho, Hq, Hp, Hr);
    const int con_tamano = f->cfg.usar_tamano && isfinite(m->tamano);
    const int nm = con_tamano ? 3 : 2;
    e->nm = nm;

    // H (nm x N), residuo y, y R con el termino de pose comun: R = diag(sigma^2, sigma^2, sigma_w^2) + J_q Sigma_q J_q^T.
    double Jq[3][2];
    memset(e->H, 0, sizeof(e->H));
    memset(e->R, 0, sizeof(e->R));
    for (int i = 0; i < 2; ++i) {
        e->H[i * N + 0] = Hp[2 * i];
        e->H[i * N + 1] = Hp[2 * i + 1];
        e->H[i * N + 6] = Hr[i];
        e->y[i] = m->z[i] - s[i];
        Jq[i][0] = Hq[2 * i];
        Jq[i][1] = Hq[2 * i + 1];
        e->R[i * nm + i] = m->sigma * m->sigma;
    }
    if (con_tamano) {
        double grad[5];
        const double hw = geo_tamano(&f->geo, m->q, psi, rho, f->cfg.kw, f->cfg.bw, grad);
        e->H[2 * N + 0] = grad[2];
        e->H[2 * N + 1] = grad[3];
        e->H[2 * N + 6] = grad[4];
        e->y[2] = m->tamano - hw;
        Jq[2][0] = grad[0];
        Jq[2][1] = grad[1];
        e->R[2 * nm + 2] = f->cfg.sigma_w * f->cfg.sigma_w;
    }
    const double q2 = f->cfg.sigma_q * f->cfg.sigma_q;
    for (int i = 0; i < nm; ++i)
        for (int j = 0; j < nm; ++j)
            e->R[i * nm + j] += q2 * (Jq[i][0] * Jq[j][0] + Jq[i][1] * Jq[j][1]);

    // S = H P H^T + R y NIS.
    double *S = e->S;
    for (int i = 0; i < N; ++i)
        for (int k = 0; k < nm; ++k) {
            double a = 0.0;
            for (int j = 0; j < N; ++j)
                a += f->P[IDX(i, j)] * e->H[k * N + j];
            e->PHt[i * nm + k] = a;
        }
    mult(e->H, e->PHt, S, nm, N, nm);
    for (int i = 0; i < nm * nm; ++i)
        S[i] += e->R[i];
    e->ok = invertir(S, nm, e->Si);
    e->nis = 0.0;
    for (int i = 0; i < nm; ++i)
        for (int j = 0; j < nm; ++j)
            e->nis += e->y[i] * e->Si[i * nm + j] * e->y[j];
    if (!e->ok)
        e->nis = INFINITY;
}
//---------------------------------------------------------------------------
// true si la medicion es valida y su NIS no supera la puerta chi^2 al 99 %.
static int pasa_la_puerta(const Evaluacion *e)
{
    return e->ok && e->nis <= (e->nm == 3 ? EKF_CHI2_3 : EKF_CHI2_2);
}
//---------------------------------------------------------------------------
// Correccion con una medicion que paso la puerta: ganancia, Joseph y cotas de rho.
static void corregir_con(EKFPieza *f, const Evaluacion *e)
{
    const int nm = e->nm;
    // K = P H^T S^-1; con rho considerado, su fila se fuerza a cero (Schmidt-Kalman).
    double K[N * 3];
    mult(e->PHt, e->Si, K, N, nm, nm);
    if (f->cfg.tratamiento_rho == RHO_CONSIDERADO)
        for (int k = 0; k < nm; ++k)
            K[6 * nm + k] = 0.0;
    for (int i = 0; i < N; ++i)
        for (int k = 0; k < nm; ++k)
            f->xi[i] += K[i * nm + k] * e->y[k];
    joseph(f, K, e->H, e->R, nm);
    f->fallos = 0;
    cotas_rho(f);
}
// NIS dividido por su valor esperado (media de la chi^2 truncada): cerca de 1 si el filtro
// esta bien calibrado.
static double nis_normalizado(double nis, int nm)
{
    return nis / (nm == 3 ? EKF_NIS_REF_3 : EKF_NIS_REF_2);
}
// Aplica una medicion ya evaluada y adapta la intensidad de maniobra con su NIS.
static void aplicar(EKFPieza *f, const Evaluacion *e)
{
    corregir_con(f, e);
    adaptar(f, nis_normalizado(e->nis, e->nm));
}
//---------------------------------------------------------------------------
// NIS de la medicion predicha combinada (momentos de la mezcla con mu predicha): con
// z_hat_j = z - y_j, el residuo es y_bar = Sigma mu_j y_j y S = Sigma mu_j [S_j + (y_j - y_bar)(y_j - y_bar)^T].
static double nis_combinado(const double mu[2], const Evaluacion e[2])
{
    if (!e[0].ok || !e[1].ok)
        return INFINITY;
    const int nm = e[0].nm;
    double y[3], S[9], Si[9];
    for (int i = 0; i < nm; ++i)
        y[i] = mu[0] * e[0].y[i] + mu[1] * e[1].y[i];
    for (int i = 0; i < nm; ++i)
        for (int k = 0; k < nm; ++k) {
            double s = 0.0;
            for (int j = 0; j < 2; ++j)
                s += mu[j] * (e[j].S[i * nm + k] + (e[j].y[i] - y[i]) * (e[j].y[k] - y[k]));
            S[i * nm + k] = s;
        }
    if (!invertir(S, nm, Si))
        return INFINITY;
    double nis = 0.0;
    for (int i = 0; i < nm; ++i)
        for (int k = 0; k < nm; ++k)
            nis += y[i] * Si[i * nm + k] * y[k];
    return nis;
}
//---------------------------------------------------------------------------
// IMM con medicion, con los modos ya interactuados y predichos hasta m->t. Cada modo se evalua
// con su prediccion y la puerta acepta si alguno la pasa: el de barrido, con su S mas grande,
// toma las maniobras sin cambiar de modo (reemplaza el re-gating). Si se acepta, se corrigen
// los dos y mu_j prop. a Lambda_j mu_j, con log Lambda_j = -(NIS_j + log det S_j)/2 (el (2pi)^nm es comun a los
// dos) normalizado por el maximo, para que un NIS enorme no desborde. El NIS reportado es el
// de la medicion predicha combinada.
static int corregir_imm(EKFPieza *f, const MedicionEKF *m, double *nis_out)
{
    EKFPieza g[2];
    Evaluacion e[2];
    int pasa = 0;
    for (int j = 0; j < 2; ++j) {
        cargar_modo(f, j, &g[j]);
        evaluar(&g[j], m, &e[j]);
        pasa |= pasa_la_puerta(&e[j]);
    }
    const double nis = nis_combinado(f->mu, e);
    if (nis_out)
        *nis_out = nis;
    if (!pasa) {
        rechazo(f, nis);
        return 0;
    }
    double lw[2], lmax = -INFINITY;
    for (int j = 0; j < 2; ++j) {
        const double ldet = log(determinante(e[j].S, e[j].nm));
        lw[j] = e[j].ok && f->mu[j] > 0.0 ? log(f->mu[j]) - 0.5 * (e[j].nis + ldet) : -INFINITY;
        lmax = fmax(lmax, lw[j]);
        if (e[j].ok) {
            corregir_con(&g[j], &e[j]);
            guardar_modo(f, j, &g[j]);
        }
    }
    if (isfinite(lmax)) {
        const double w0 = exp(lw[0] - lmax), w1 = exp(lw[1] - lmax);
        f->mu[0] = w0 / (w0 + w1);
        f->mu[1] = w1 / (w0 + w1);
    }
    combinar(f);
    f->fallos = 0;
    adaptar(f, nis_normalizado(nis, e[0].nm));
    return 1;
}
//---------------------------------------------------------------------------
// Si una medicion no pasa la puerta, el rechazo sube phi_k o cambia al modo de barrido (la
// hipotesis de maniobra). La misma medicion se vuelve a evaluar una vez con esa hipotesis,
// predicha desde el ultimo posterior: si no, la medicion que delato la maniobra se perderia
// y las siguientes, con la prediccion cada vez mas lejos, tambien (cascada de rechazos).
int ekf_corregir(EKFPieza *f, const MedicionEKF *m, double *nis_out)
{
    if (nis_out)
        *nis_out = NAN;
    if (!f->inicializado) {
        const int r = arrancar(f, m);
        if (f->inicializado && f->cfg.seleccion == SELECCION_IMM)
            iniciar_modos(f);
        return r;
    }
    if (f->cfg.seleccion == SELECCION_IMM) {
        ekf_predecir(f, m->t);
        return corregir_imm(f, m, nis_out);
    }
    const EKFPieza previo = *f;
    ekf_predecir(f, m->t);
    Evaluacion e;
    evaluar(f, m, &e);
    if (nis_out)
        *nis_out = e.nis;
    if (pasa_la_puerta(&e)) {
        aplicar(f, &e);
        return 1;
    }
    rechazo(f, e.nis);
    if (!f->inicializado || (f->modo == previo.modo && f->phi_k <= previo.phi_k))
        return 0;
    const int modo = f->modo, fallos = f->fallos;
    const double phi = f->phi_k;
    *f = previo;
    f->phi_k = phi;
    f->fallos = fallos;
    if (modo != previo.modo) {
        f->modo = modo;
        prior_de_maniobra(f);
        if (f->cfg.histeresis_modo)
            vaciar_ventana(f);
    }
    ekf_predecir(f, m->t);
    evaluar(f, m, &e);
    if (pasa_la_puerta(&e)) {
        if (nis_out)
            *nis_out = e.nis;
        aplicar(f, &e);
        if (modo != previo.modo && f->cfg.histeresis_modo)
            vaciar_ventana(f); // la que delato la maniobra se evaluo con el prior: no cuenta
        return 1;
    }
    inflar(f);
    return 0;
}
//---------------------------------------------------------------------------
// Sin deteccion: solo prediccion (informe: "se predice sin corregir"); tras max_fallos, o si
// el blanco quedo perdido, se reinicia psi conservando rho.
void ekf_sin_medicion(EKFPieza *f, double t)
{
    if (!f->inicializado) {
        if (f->hay_pendiente && ++f->fallos > f->cfg.max_fallos)
            ekf_reiniciar(f);
        return;
    }
    ekf_predecir(f, t);
    if (++f->fallos > f->cfg.max_fallos || perdido(f))
        ekf_reiniciar(f);
}

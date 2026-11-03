// Detector y seguidor CFAR de blancos puntuales. Ver spottracker.hpp para la explicacion
// general del metodo.
//
// Todo el trabajo que se hace pixel por pixel esta en dos lazos: extraer() (que arma el
// canal y las imagenes integrales) y cfar() (que hace cinco consultas de caja por pixel,
// y esta escrito para que el compilador lo vectorice en el interior de la imagen). No se
// pide memoria dinamica mientras el seguidor funciona: los buffers se dimensionan una vez
// y se reutilizan.
//
// Que es una imagen integral? Es una tabla S donde S(x, y) guarda la suma de todos los
// pixeles que estan arriba y a la izquierda de (x, y). Con ella, la suma de cualquier
// rectangulo se obtiene con solo 4 lecturas:
//     suma = S(x1, y1) - S(x0, y1) - S(x1, y0) + S(x0, y0)
// (se toma todo lo que esta arriba a la izquierda de la esquina inferior derecha, se le
// quitan las dos franjas que sobran, y se devuelve la esquina que se quito dos veces).
// Ademas se guarda Q, la integral de los cuadrados de los pixeles. Con S y Q se obtienen
// la media y la varianza de cualquier rectangulo: media = suma / n, y
// varianza = (suma de cuadrados / n) - media^2.

#include "spottracker.hpp"

#include <algorithm>
#include <cmath>

// Funciones auxiliares, visibles solo dentro de este archivo.
namespace {

// Las integrales van en uint32 con aritmetica modular: una suma de caja sale exacta
// aunque la integral completa desborde, siempre que la caja misma quepa. La mas grande
// es la de cuadrados del anillo exterior, (2t+1)^2 * 255^2, y para poder pasarla por
// int32 a float (conversion vectorizable) tiene que ser < 2^31: lado <= 181 px, o sea
// guarda + anillo <= 90 pixeles de trabajo.
//
// Explicado con mas calma: en una imagen grande, la suma total de los cuadrados de los
// pixeles no cabe en 32 bits y "da la vuelta" (desborda). Pero las restas de la formula
// de la caja tambien dan la vuelta de la misma manera, asi que los desbordes se cancelan
// y el resultado final es correcto, siempre que el valor verdadero de la caja quepa. El
// peor caso es la caja exterior del anillo llena de pixeles blancos (255^2 cada uno).
const int kSemiladoMax = 90;

// --- canal --------------------------------------------------------------------------

// Valor del canal elegido para un pixel con componentes (b, g, r).
//
// El canal C es un parametro de "template": se fija al compilar, no al ejecutar. Asi, en
// vez de tener un switch que se evalua en cada uno de los miles de pixeles, el compilador
// genera una version de la funcion por cada canal, cada una con su cuenta directa y sin
// decisiones adentro.
template <int C>
inline int valorBGR(int b, int g, int r)
{
    switch (C)
    {
    case SpotTracker::ROJO: return r;
    case SpotTracker::VERDE: return g;
    case SpotTracker::AZUL: return b;
    // Los canales de croma dicen cuanto "sobresale" un color por encima de los otros dos.
    // Un pixel blanco o gris (los tres iguales) da 0, y uno rojo puro da mucho.
    case SpotTracker::CROMA_ROJO: return std::max(r - std::max(g, b), 0);
    case SpotTracker::CROMA_VERDE: return std::max(g - std::max(r, b), 0);
    case SpotTracker::CROMA_AZUL: return std::max(b - std::max(r, g), 0);
    // Luma aproximada, sin multiplicaciones: (b + 2g + r) / 4, con pesos de 1/4, 1/2 y
    // 1/4 (el verde pesa mas, igual que en la percepcion humana del brillo). El +2 sirve
    // para redondear, y ">> 2" divide por 4 desplazando bits, que es mas rapido.
    default: return (b + 2 * g + r + 2) >> 2;   // luma aproximada, sin multiplicaciones
    }
}
//---------------------------------------------------------------------------
// Calcula una fila (y) del canal a partir de una imagen BGR, promediando bloques de d x d
// pixeles (submuestreo). Los casos d = 1 y d = 2 (los de uso real) estan escritos aparte
// para recorrer las filas con punteros, que es mas rapido; el caso general sirve para
// cualquier d. "inv" invierte el canal (255 - v) para blancos mas oscuros que el fondo.
//
// Recordar que en una imagen BGR cada pixel ocupa 3 bytes seguidos (azul, verde, rojo),
// por eso los punteros avanzan de a 3 (o de a 6 cuando se toman dos pixeles a la vez).
template <int C>
void filaBGR(const cv::Mat &img, const cv::Rect &r, int d, int y, int W, bool inv, uint8_t *c)
{
    if (d == 1)
    {
        const uchar *p = img.ptr<uchar>(r.y + y) + 3 * r.x;
        for (int x = 0; x < W; x++, p += 3)
        {
            const int v = valorBGR<C>(p[0], p[1], p[2]);
            c[x] = (uint8_t)(inv ? 255 - v : v);
        }
    }
    else if (d == 2)
    {
        // p y q apuntan a las dos filas de la imagen que forman una fila de trabajo. Cada
        // componente se promedia sobre el bloque de 2x2 (el +2 redondea y >> 2 divide por 4).
        const uchar *p = img.ptr<uchar>(r.y + 2 * y) + 3 * r.x;
        const uchar *q = img.ptr<uchar>(r.y + 2 * y + 1) + 3 * r.x;
        for (int x = 0; x < W; x++, p += 6, q += 6)
        {
            const int b = (p[0] + p[3] + q[0] + q[3] + 2) >> 2;
            const int g = (p[1] + p[4] + q[1] + q[4] + 2) >> 2;
            const int rr = (p[2] + p[5] + q[2] + q[5] + 2) >> 2;
            const int v = valorBGR<C>(b, g, rr);
            c[x] = (uint8_t)(inv ? 255 - v : v);
        }
    }
    else
    {
        // Caso general: se suman los d x d pixeles del bloque y se divide por d*d,
        // redondeando (sumar dd / 2 antes de dividir equivale a redondear).
        const int dd = d * d;
        for (int x = 0; x < W; x++)
        {
            int sb = 0, sg = 0, sr = 0;
            for (int k = 0; k < d; k++)
            {
                const uchar *p = img.ptr<uchar>(r.y + y * d + k) + 3 * (r.x + x * d);
                for (int j = 0; j < d; j++, p += 3)
                {
                    sb += p[0];
                    sg += p[1];
                    sr += p[2];
                }
            }
            const int v = valorBGR<C>((sb + dd / 2) / dd, (sg + dd / 2) / dd, (sr + dd / 2) / dd);
            c[x] = (uint8_t)(inv ? 255 - v : v);
        }
    }
}
//---------------------------------------------------------------------------
// Lo mismo que filaBGR, pero para una imagen en escala de grises (1 byte por pixel).
void filaGris(const cv::Mat &img, const cv::Rect &r, int d, int y, int W, bool inv, uint8_t *c)
{
    const int dd = d * d;
    for (int x = 0; x < W; x++)
    {
        int v = 0;
        for (int k = 0; k < d; k++)
        {
            const uchar *p = img.ptr<uchar>(r.y + y * d + k) + r.x + x * d;
            for (int j = 0; j < d; j++)
                v += p[j];
        }
        v = (v + dd / 2) / dd;
        c[x] = (uint8_t)(inv ? 255 - v : v);
    }
}
//---------------------------------------------------------------------------
// Canal de la region `r` completa, fila por fila, y a continuacion la fila de las dos
// integrales (la suma acumulada es serial, la extraccion del canal no).
//
// C < 0 indica una imagen en escala de grises. El truco filaBGR<(C < 0 ? 0 : C)> existe
// porque, aunque el "if" elija la rama de grises, el compilador igual compila la otra
// rama, y ahi necesita un valor de canal valido. Asi nunca se crea una filaBGR<-1>.
//
// Para las integrales se usa la relacion S(y+1, x+1) = S(y, x+1) + (suma de la fila y
// hasta x): la fila de abajo de la integral es la de arriba mas la suma acumulada de la
// fila actual (fs para S y fq para Q).
template <int C>
void extraerCanal(const cv::Mat &img, const cv::Rect &r, int d, bool inv, int W, int H,
                  uint8_t *ch, uint32_t *S, uint32_t *Q)
{
    const int W1 = W + 1; // las integrales tienen una columna (y una fila) extra de ceros
    for (int y = 0; y < H; y++)
    {
        uint8_t *c = ch + (size_t)y * W;
        if (C < 0)
            filaGris(img, r, d, y, W, inv, c);
        else
            filaBGR<(C < 0 ? 0 : C)>(img, r, d, y, W, inv, c);

        // Sa y Qa: fila y de las integrales (la de arriba). Sb y Qb: fila y + 1 (la que
        // se esta calculando).
        const uint32_t *Sa = S + (size_t)y * W1, *Qa = Q + (size_t)y * W1;
        uint32_t *Sb = S + (size_t)(y + 1) * W1, *Qb = Q + (size_t)(y + 1) * W1;
        uint32_t fs = 0, fq = 0;
        for (int x = 0; x < W; x++)
        {
            const uint32_t v = c[x];
            fs += v;
            fq += v * v;
            Sb[x + 1] = Sa[x + 1] + fs;
            Qb[x + 1] = Qa[x + 1] + fq;
        }
    }
}

// --- CFAR ---------------------------------------------------------------------------

// Mejor pixel encontrado por el CFAR, con las estadisticas del fondo a su alrededor.
struct Pico
{
    float s2;    // snr^2 (se trabaja con el cuadrado para no calcular raices)
    int x, y;    // coordenadas de trabajo
    float mu, var; // media y varianza del anillo en ese pixel
};
//---------------------------------------------------------------------------
// Suma de la caja [x0, x1) x [y0, y1) usando la integral I (ver la formula al comienzo del
// archivo). W1 es el ancho de la integral. Las restas se hacen en uint32 (aritmetica
// modular) y recien el resultado se pasa a int32.
inline int32_t caja(const uint32_t *I, int W1, int x0, int y0, int x1, int y1)
{
    return (int32_t)((I[y1 * W1 + x1] - I[y0 * W1 + x1]) - (I[y1 * W1 + x0] - I[y0 * W1 + x0]));
}
//---------------------------------------------------------------------------
// snr^2 de un pixel con las ventanas recortadas contra el borde de la region (bordes de
// la imagen, donde el anillo queda asimetrico pero sigue siendo valido).
//
// c, g y t son los semilados de la CUT, de la guarda y del borde exterior del anillo. Las
// tres cajas se recortan a la region, y el anillo se obtiene como "caja exterior menos
// caja de la guarda". Si mu_out no es nulo, tambien se devuelven la media y la varianza
// del anillo.
float cfarRecortado(const uint32_t *S, const uint32_t *Q, int W, int H, int x, int y, int c,
                    int g, int t, float var_min, float *mu_out, float *var_out)
{
    const int W1 = W + 1;
    // Cajas recortadas: CUT (c), guarda (g) y exterior del anillo (o).
    const int cx0 = std::max(x - c, 0), cx1 = std::min(x + c + 1, W);
    const int cy0 = std::max(y - c, 0), cy1 = std::min(y + c + 1, H);
    const int gx0 = std::max(x - g, 0), gx1 = std::min(x + g + 1, W);
    const int gy0 = std::max(y - g, 0), gy1 = std::min(y + g + 1, H);
    const int ox0 = std::max(x - t, 0), ox1 = std::min(x + t + 1, W);
    const int oy0 = std::max(y - t, 0), oy1 = std::min(y + t + 1, H);
    const int nc = (cx1 - cx0) * (cy1 - cy0);                              // pixeles de la CUT
    const int nr = (ox1 - ox0) * (oy1 - oy0) - (gx1 - gx0) * (gy1 - gy0);  // pixeles del anillo
    // Con menos de 8 pixeles en el anillo, la estadistica del fondo no es confiable.
    if (nr < 8)
        return 0.0f;
    const float inv = 1.0f / (float)nr;
    const float mu = (float)(caja(S, W1, ox0, oy0, ox1, oy1) - caja(S, W1, gx0, gy0, gx1, gy1)) * inv;
    const float var = std::max(
        (float)(caja(Q, W1, ox0, oy0, ox1, oy1) - caja(Q, W1, gx0, gy0, gx1, gy1)) * inv - mu * mu,
        var_min);
    const float dif = (float)caja(S, W1, cx0, cy0, cx1, cy1) / (float)nc - mu; // media(CUT) - mu
    if (mu_out)
    {
        *mu_out = mu;
        *var_out = var;
    }
    // Solo cuentan los pixeles MAS brillantes que el fondo (dif > 0).
    return dif > 0.0f ? dif * dif / var : 0.0f;
}
//---------------------------------------------------------------------------
// CA-CFAR en [x0,x1) x [y0,y1) de la region de trabajo. Devuelve el pico; si `mapa` no
// es nulo guarda ahi snr^2 (0 bajo s2_min). `fila` es un buffer de al menos W floats.
//
// Cada fila se hace en dos pasadas: una aritmetica pura sobre el interior (sin recortes,
// conteos constantes, sin ramas: el compilador la vectoriza) y un barrido escalar que
// busca el maximo. Solo los pocos pixeles del borde pagan los recortes.
//
// Parametros de la geometria (en pixeles de trabajo): c es el semilado de la CUT, g el de
// la guarda y a el espesor del anillo. var_min es el piso de la varianza y s2_min el
// umbral (al cuadrado).
Pico cfar(const uint32_t *S, const uint32_t *Q, int W, int H, int x0, int y0, int x1, int y1,
          int c, int g, int a, float var_min, float s2_min, float *mapa, int paso_mapa,
          float *fila)
{
    var_min = std::max(var_min, 1e-6f);   // var > 0 siempre: s2 * 0 nunca es NaN
    // Se acota la geometria: anillo de hasta 10, guarda + anillo de hasta kSemiladoMax, y
    // la CUT nunca mas grande que la guarda.
    a = std::min(a, 10);
    g = std::min(g, kSemiladoMax - a);
    c = std::min(c, g);
    const int W1 = W + 1;
    const int t = g + a;                  // semilado del borde exterior del anillo
    Pico mejor = {0.0f, -1, -1, 0.0f, 0.0f};

    for (int y = y0; y < y1; y++)
    {
        // Donde se escribe el snr^2 de esta fila: en el mapa (si se pidio) o en el buffer
        // de una fila. Se le resta x0 al puntero para que out[x] se pueda indexar
        // directamente con la coordenada x (sin tener que escribir out[x - x0]).
        float *out = (mapa ? mapa + (size_t)(y - y0) * paso_mapa : fila) - x0;
        // Rango de x donde el anillo completo cabe dentro de la region (el "interior").
        const int xi0 = std::max(x0, t), xi1 = std::min(x1, W - t);
        const bool interior_y = y - t >= 0 && y + t + 1 <= H;

        if (interior_y && xi0 < xi1)
        {
            // Punteros a las filas de las integrales que se necesitan (arriba y abajo de
            // cada caja). Como las cajas tienen siempre el mismo tamano en el interior,
            // basta con precalcular estas filas y las cantidades de pixeles.
            const uint32_t *Sc0 = S + (size_t)(y - c) * W1, *Sc1 = S + (size_t)(y + c + 1) * W1;
            const uint32_t *Sg0 = S + (size_t)(y - g) * W1, *Sg1 = S + (size_t)(y + g + 1) * W1;
            const uint32_t *So0 = S + (size_t)(y - t) * W1, *So1 = S + (size_t)(y + t + 1) * W1;
            const uint32_t *Qg0 = Q + (size_t)(y - g) * W1, *Qg1 = Q + (size_t)(y + g + 1) * W1;
            const uint32_t *Qo0 = Q + (size_t)(y - t) * W1, *Qo1 = Q + (size_t)(y + t + 1) * W1;
            const float inv_nc = 1.0f / (float)((2 * c + 1) * (2 * c + 1));
            const float inv_nr = 1.0f / (float)((2 * t + 1) * (2 * t + 1) - (2 * g + 1) * (2 * g + 1));
            for (int x = xi0; x < xi1; x++)
            {
                // Sumas de las cajas: CUT (sc), exterior (so) y guarda (sg), y las de
                // cuadrados del exterior (qo) y de la guarda (qg).
                const int32_t sc = (int32_t)((Sc1[x + c + 1] - Sc0[x + c + 1]) - (Sc1[x - c] - Sc0[x - c]));
                const int32_t so = (int32_t)((So1[x + t + 1] - So0[x + t + 1]) - (So1[x - t] - So0[x - t]));
                const int32_t sg = (int32_t)((Sg1[x + g + 1] - Sg0[x + g + 1]) - (Sg1[x - g] - Sg0[x - g]));
                const int32_t qo = (int32_t)((Qo1[x + t + 1] - Qo0[x + t + 1]) - (Qo1[x - t] - Qo0[x - t]));
                const int32_t qg = (int32_t)((Qg1[x + g + 1] - Qg0[x + g + 1]) - (Qg1[x - g] - Qg0[x - g]));
                // Anillo = exterior - guarda.
                const float mu = (float)(so - sg) * inv_nr;
                const float var = std::max((float)(qo - qg) * inv_nr - mu * mu, var_min);
                const float dif = (float)sc * inv_nc - mu;
                const float s2 = dif * dif / var;
                // Sin ramas (& y no &&): un salto dentro del lazo impide vectorizarlo.
                // Multiplicar por el resultado de la comparacion (0 o 1) equivale a
                // "si no cumple, poner 0", pero sin usar un if.
                out[x] = s2 * (float)((dif > 0.0f) & (s2 >= s2_min));
            }
            // Los pixeles de los bordes izquierdo y derecho, con cajas recortadas.
            for (int x = x0; x < xi0; x++)
            {
                const float s2 = cfarRecortado(S, Q, W, H, x, y, c, g, t, var_min, nullptr, nullptr);
                out[x] = s2 >= s2_min ? s2 : 0.0f;
            }
            for (int x = xi1; x < x1; x++)
            {
                const float s2 = cfarRecortado(S, Q, W, H, x, y, c, g, t, var_min, nullptr, nullptr);
                out[x] = s2 >= s2_min ? s2 : 0.0f;
            }
        }
        else
        {
            // Fila cerca del borde de arriba o de abajo: todos sus pixeles con recorte.
            for (int x = x0; x < x1; x++)
            {
                const float s2 = cfarRecortado(S, Q, W, H, x, y, c, g, t, var_min, nullptr, nullptr);
                out[x] = s2 >= s2_min ? s2 : 0.0f;
            }
        }

        // Segunda pasada de la fila: busqueda del maximo.
        for (int x = x0; x < x1; x++)
            if (out[x] > mejor.s2)
            {
                mejor.s2 = out[x];
                mejor.x = x;
                mejor.y = y;
            }
    }

    // Estadisticas del fondo en el pico (hacen falta para el nivel de corte del blob).
    if (mejor.x >= 0)
        cfarRecortado(S, Q, W, H, mejor.x, mejor.y, c, g, t, var_min, &mejor.mu, &mejor.var);
    return mejor;
}
//---------------------------------------------------------------------------
// Redondeo hacia arriba, de float a int.
inline int techo(float v) { return (int)std::ceil(v); }

} // namespace

// =====================================================================================

// Valores por defecto de los parametros.
SpotTracker::Params::Params()
    : canal(GRIS),
      oscuro(false),
      umbral_adquisicion(10.0f),
      umbral_seguimiento(5.0f),
      sigma_min(2.0f),
      contraste_min(0.0f),
      celda(1),
      guarda_min(4),
      anillo(4),
      radio_blob_max(10.0f),
      decimacion(2),
      max_candidatos(6),
      vel_adquisicion(20.0f),
      radio_min(16.0f),
      radio_max(160.0f),
      k_vel(2.0f),
      crecimiento(1.5f),
      retencion_rapidez(0.9f),
      radio_grueso(40.0f),
      confirmaciones(2),
      ambiguedad(0.0f),
      max_fallos(8),
      alfa(0.85f),
      beta(0.5f)
{
}
//---------------------------------------------------------------------------
SpotTracker::SpotTracker(const Params &p) : params(p), _sugerencia(false), _sug_radio(0.0f)
{
    reset();
}
//---------------------------------------------------------------------------
// Vuelve al estado inicial. _radio_blob arranca en 2 px (un punto chico) hasta medir uno.
void SpotTracker::reset()
{
    _estado = BUSCANDO;
    _pos = _vel = cv::Point2f(0.0f, 0.0f);
    _aciertos = _fallos = 0;
    _detectado = false;
    _det = Deteccion();
    _radio_blob = 2.0f;
    _rapidez = 0.0f;
    _ventana = cv::Rect();
    _rechazo = NINGUNO;
}
//---------------------------------------------------------------------------
// Solo se entrega medicion cuando el blanco esta confirmado (SIGUIENDO) y ademas se lo
// vio en este cuadro. Una posicion solo predicha no sirve como medicion.
bool SpotTracker::medicion(cv::Point2f &z) const
{
    if (_estado != SIGUIENDO || !_detectado)
        return false;
    z = _det.centro;
    return true;
}

// ------------------------------------------------------------------------------------

// Prepara la region de trabajo t a partir del rectangulo r de la imagen, con submuestreo
// d: calcula su tamano, se asegura de que los buffers alcancen, pone en cero la primera
// fila y la primera columna de las integrales (el "borde" de ceros que necesita la
// formula de la caja) y extrae el canal con la version de extraerCanal que corresponda.
void SpotTracker::extraer(const cv::Mat &frame, const cv::Rect &r, int d, Trabajo &t) const
{
    t.d = d;
    t.origen = r.tl();
    t.W = r.width / d;
    t.H = r.height / d;
    // Los buffers solo crecen (nunca se achican), asi que despues de los primeros
    // cuadros ya no se vuelve a pedir memoria.
    const size_t n1 = (size_t)(t.W + 1) * (t.H + 1);
    if (t.ch.size() < (size_t)t.W * t.H)
        t.ch.resize((size_t)t.W * t.H);
    if (t.S.size() < n1)
    {
        t.S.resize(n1);
        t.Q.resize(n1);
    }
    std::fill(t.S.begin(), t.S.begin() + t.W + 1, 0);   // fila 0 de la integral
    std::fill(t.Q.begin(), t.Q.begin() + t.W + 1, 0);
    for (int y = 1; y <= t.H; y++)                       // columna 0
    {
        t.S[(size_t)y * (t.W + 1)] = 0;
        t.Q[(size_t)y * (t.W + 1)] = 0;
    }

    uint8_t *ch = t.ch.data();
    uint32_t *S = t.S.data();
    uint32_t *Q = t.Q.data();
    const bool inv = params.oscuro;
    // Se elige la version de extraerCanal segun el canal (-1 = imagen en grises). Este
    // switch se evalua una sola vez por region, no una vez por pixel.
    const int C = frame.channels() == 1 ? -1 : (int)params.canal;
    switch (C)
    {
    case -1: extraerCanal<-1>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case ROJO: extraerCanal<ROJO>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case VERDE: extraerCanal<VERDE>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case AZUL: extraerCanal<AZUL>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case CROMA_ROJO: extraerCanal<CROMA_ROJO>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case CROMA_VERDE: extraerCanal<CROMA_VERDE>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    case CROMA_AZUL: extraerCanal<CROMA_AZUL>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    default: extraerCanal<GRIS>(frame, r, d, inv, t.W, t.H, ch, S, Q); break;
    }
}
//---------------------------------------------------------------------------
// CFAR a resolucion completa en una ventana de semilado `radio` alrededor de `centro`,
// y verificacion del blob en el pico: centroide subpixel, area y que no se salga de la
// guarda (si se sale, es algo extendido y no un punto).
bool SpotTracker::detectarEnVentana(const cv::Mat &frame, cv::Point2f centro, float radio,
                                    int guarda, float umbral, Deteccion &out)
{
    const cv::Rect imagen(0, 0, frame.cols, frame.rows);
    // Ventana de busqueda: cuadrado centrado en "centro" (redondeado al pixel mas cercano),
    // recortado a la imagen.
    const int r = std::max(1, techo(radio));
    const int cx = (int)std::floor(centro.x + 0.5f), cy = (int)std::floor(centro.y + 0.5f);
    const cv::Rect busq = cv::Rect(cx - r, cy - r, 2 * r + 1, 2 * r + 1) & imagen;
    _ventana = busq;
    if (busq.area() == 0)
    {
        _rechazo = FUERA_DE_IMAGEN;
        return false;
    }

    // La region de trabajo es la ventana mas un margen de guarda + anillo por cada lado,
    // porque para evaluar un pixel del borde de la ventana hace falta su anillo completo.
    const int margen = guarda + params.anillo;
    const cv::Rect region = cv::Rect(busq.x - margen, busq.y - margen,
                                     busq.width + 2 * margen, busq.height + 2 * margen) & imagen;
    extraer(frame, region, 1, _local);
    const Trabajo &t = _local;

    // La ventana, en coordenadas de la region.
    const int x0 = busq.x - region.x, y0 = busq.y - region.y;
    if (_fila.size() < (size_t)t.W)
        _fila.resize((size_t)t.W);
    const Pico p = cfar(t.S.data(), t.Q.data(), t.W, t.H, x0, y0, x0 + busq.width,
                        y0 + busq.height, params.celda, guarda, params.anillo,
                        params.sigma_min * params.sigma_min, umbral * umbral, nullptr, 0,
                        _fila.data());
    if (p.x < 0 || p.s2 < umbral * umbral)
    {
        _rechazo = SNR_BAJO;
        return false;
    }

    // --- verificacion del blob -------------------------------------------------------
    // Se mide solo la componente conexa (4-vecinos) que contiene al pico, dentro de una
    // ventana de semilado `guarda`: lo demas que haya en la ventana (textura, otro objeto)
    // no la contamina, y el costo es proporcional al tamano del blob. El nivel de corte
    // es relativo al fondo local (mu, sigma del anillo) y al propio pico, asi que tambien
    // es invariante a la iluminacion.
    //
    // "Componente conexa" es la mancha de pixeles brillantes pegados entre si que contiene
    // al pico. "4-vecinos" significa que dos pixeles se consideran pegados si se tocan por
    // un lado (arriba, abajo, izquierda o derecha), no solo por una esquina.
    const int bx0 = std::max(p.x - guarda, 0), bx1 = std::min(p.x + guarda + 1, t.W);
    const int by0 = std::max(p.y - guarda, 0), by1 = std::min(p.y + guarda + 1, t.H);
    const int bw = bx1 - bx0, bh = by1 - by0;
    const uint8_t *ch = t.ch.data();

    // Maximo cerca del pico (no en toda la ventana: una lampara saturada lo inflaria).
    // Ese pixel mas brillante es la "semilla" desde la que se rellena la mancha.
    const int rv = params.celda + 2;
    int vmax = 0, semilla = -1;
    for (int y = std::max(p.y - rv, by0); y < std::min(p.y + rv + 1, by1); y++)
        for (int x = std::max(p.x - rv, bx0); x < std::min(p.x + rv + 1, bx1); x++)
            if (ch[(size_t)y * t.W + x] > vmax)
            {
                vmax = ch[(size_t)y * t.W + x];
                semilla = (y - by0) * bw + (x - bx0);
            }
    // Nivel de corte: un pixel es parte de la mancha si supera el fondo por al menos 3
    // sigmas, o por el 30 % de la altura del pico, lo que sea mayor.
    const float sigma = std::sqrt(p.var);
    const float corte = p.mu + std::max(3.0f * sigma, 0.3f * ((float)vmax - p.mu));
    if (semilla < 0 || (float)vmax <= corte || (float)vmax - p.mu < params.contraste_min)
    {
        _rechazo = SNR_BAJO;
        return false;
    }

    // Un lado de la ventana solo cuenta como "borde" si no es el borde de la imagen: un
    // blanco parcialmente fuera del cuadro sigue siendo valido.
    const bool bl = p.x - guarda >= 0, br = p.x + guarda + 1 <= t.W;
    const bool bt = p.y - guarda >= 0, bb = p.y + guarda + 1 <= t.H;

    // Relleno ("flood fill") con una pila: se parte de la semilla y se van agregando los
    // vecinos que superan el corte y que no se habian visitado (_marca los recuerda).
    if (_marca.size() < (size_t)bw * bh)
        _marca.resize((size_t)bw * bh);
    std::fill(_marca.begin(), _marca.begin() + (size_t)bw * bh, 0);
    _pila.clear();
    _pila.push_back(semilla);
    _marca[semilla] = 1;

    // Acumuladores. sw, swx y swy sirven para el centroide ponderado por brillo (los
    // pixeles mas brillantes pesan mas). sx, sy, sxx, syy y sxy son los momentos
    // "binarios" (todos los pixeles pesan igual), que describen la forma de la mancha.
    double sw = 0.0, swx = 0.0, swy = 0.0;
    int64_t sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;   // momentos binarios, para la forma
    int area = 0, mx0 = bw, mx1 = -1, my0 = bh, my1 = -1;
    bool toca = false; // la mancha llego al borde de la ventana de guarda
    while (!_pila.empty())
    {
        const int i = _pila.back();
        _pila.pop_back();
        const int x = i % bw, y = i / bw;   // coordenadas dentro de la ventana
        const float w = (float)ch[(size_t)(y + by0) * t.W + x + bx0] - corte; // peso = brillo sobre el corte
        sw += w;
        swx += w * x;
        swy += w * y;
        sx += x; sy += y;
        sxx += x * x; syy += y * y; sxy += x * y;
        area++;
        mx0 = std::min(mx0, x); mx1 = std::max(mx1, x);
        my0 = std::min(my0, y); my1 = std::max(my1, y);
        if ((x == 0 && bl) || (x == bw - 1 && br) || (y == 0 && bt) || (y == bh - 1 && bb))
            toca = true;

        // Se revisan los 4 vecinos: izquierda, derecha, arriba y abajo.
        const int vec[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (int k = 0; k < 4; k++)
        {
            const int nx = x + vec[k][0], ny = y + vec[k][1];
            if (nx < 0 || nx >= bw || ny < 0 || ny >= bh)
                continue;
            const int j = ny * bw + nx;
            if (_marca[j] || (float)ch[(size_t)(ny + by0) * t.W + nx + bx0] <= corte)
                continue;
            _marca[j] = 1;
            _pila.push_back(j);
        }
    }
    // Si la mancha llega al borde de la guarda, es mas grande de lo que la guarda puede
    // contener: es algo extendido (una lampara, un borde brillante), no un punto.
    if (toca)
    {
        _rechazo = EXTENDIDO;
        return false;
    }

    // Resultado, pasado de coordenadas de la ventana a coordenadas de la imagen completa.
    const float ox = (float)(region.x + bx0), oy = (float)(region.y + by0);
    out.centro = cv::Point2f((float)(swx / sw) + ox, (float)(swy / sw) + oy);
    out.caja = cv::Rect(mx0 + (int)ox, my0 + (int)oy, mx1 - mx0 + 1, my1 - my0 + 1);
    out.snr = std::sqrt(p.s2);
    out.area = area;
    // Ejes de la elipse equivalente (desviaciones a lo largo de los ejes principales).
    // Se arma la matriz de covarianza de las posiciones de los pixeles de la mancha
    // [cxx cxy; cxy cyy], y sus dos valores propios son las varianzas a lo largo del eje
    // mayor y del menor (se usa la misma formula cerrada de 2x2 que en lazo.cpp).
    // El +1/12 es la varianza de un pixel "de verdad", que es un cuadradito de lado 1 y no
    // un punto: asi una mancha de un solo pixel no da ejes de tamano cero.
    const double mxm = (double)sx / area, mym = (double)sy / area;
    const double cxx = (double)sxx / area - mxm * mxm + 1.0 / 12.0;   // +1/12: pixel finito
    const double cyy = (double)syy / area - mym * mym + 1.0 / 12.0;
    const double cxy = (double)sxy / area - mxm * mym;
    const double m = 0.5 * (cxx + cyy);
    const double dm = std::sqrt(0.25 * (cxx - cyy) * (cxx - cyy) + cxy * cxy);
    out.eje_mayor = (float)std::sqrt(m + dm);
    out.eje_menor = (float)std::sqrt(std::max(m - dm, 0.0));
    _rechazo = NINGUNO;
    return true;
}
//---------------------------------------------------------------------------
// Busqueda gruesa a fina dentro de `zona`: CFAR submuestreado, maximos locales, y los
// mejores se verifican a resolucion completa con detectarEnVentana(). Es la busqueda
// global (zona = imagen entera) y tambien la de seguimiento cuando la ventana es grande.
//
// guarda_px es la guarda en pixeles de imagen, v_lim la estela admitida para la
// verificacion de forma, y pred (si no es nulo) una posicion preferida: los candidatos
// mas cercanos a ella se prueban primero.
bool SpotTracker::buscarGrueso(const cv::Mat &frame, const cv::Rect &zona, int guarda_px,
                               float umbral, float v_lim, const cv::Point2f *pred,
                               Deteccion &out, std::vector<Deteccion> *todas,
                               const cv::Point2f *excluir, float radio_excl)
{
    const int d = std::max(1, params.decimacion);
    const cv::Rect imagen(0, 0, frame.cols, frame.rows);
    // Region = zona + margen para los anillos, recortada a la imagen y ajustada a un
    // multiplo de d (para que los bloques de submuestreo queden completos).
    const int margen = guarda_px + params.anillo;
    cv::Rect region = cv::Rect(zona.x - margen, zona.y - margen, zona.width + 2 * margen,
                               zona.height + 2 * margen) & imagen;
    region.width = (region.width / d) * d;
    region.height = (region.height / d) * d;
    if (region.area() == 0)
    {
        _rechazo = FUERA_DE_IMAGEN;
        return false;
    }
    extraer(frame, region, d, _global);
    const Trabajo &t = _global;

    // Geometria en pixeles de trabajo (dividiendo por d). El anillo nunca baja de 2,
    // para que la estadistica del fondo tenga suficientes pixeles.
    const int c = params.celda / d;
    const int g = techo((float)guarda_px / d);
    const int a = std::max(2, techo((float)params.anillo / d));

    // Parte de la region que corresponde a la zona (sin el margen del anillo).
    const cv::Rect z = cv::Rect((zona.x - region.x) / d, (zona.y - region.y) / d,
                                techo((float)zona.width / d), techo((float)zona.height / d)) &
                       cv::Rect(0, 0, t.W, t.H);
    if (z.area() == 0)
    {
        _rechazo = FUERA_DE_IMAGEN;
        return false;
    }
    if (_mapa.size() < (size_t)z.area())
        _mapa.resize((size_t)z.area());

    // El submuestreo promedia un punto chico con fondo y baja su snr: se preselecciona
    // con la mitad del umbral y la decision final se toma a resolucion completa.
    // (Promediar d x d pixeles tambien reduce la varianza del ruido d*d veces, por eso el
    // piso de varianza se divide por d*d).
    const float var_min = params.sigma_min * params.sigma_min / (float)(d * d);
    const float u = 0.5f * umbral;
    const Pico global = cfar(t.S.data(), t.Q.data(), t.W, t.H, z.x, z.y, z.x + z.width,
                             z.y + z.height, c, g, a, var_min, u * u, _mapa.data(), z.width,
                             nullptr);
    if (global.s2 < u * u)
    {
        _rechazo = SNR_BAJO;
        return false;
    }

    // Maximos locales 3x3 sobre el umbral, los kBrutos mas fuertes (insercion en un arreglo
    // chico). La comparacion es estricta hacia arriba y a la izquierda y no estricta hacia
    // abajo y a la derecha: una meseta da un solo maximo y no uno por pixel.
    //
    // Un "maximo local" es un pixel mayor que sus 8 vecinos. Los candidatos se mantienen
    // ordenados de mayor a menor en cs (con sus coordenadas en cxs y cys): cada candidato
    // nuevo se inserta en su lugar corriendo hacia atras a los mas debiles, y si la lista
    // esta llena, el mas debil se descarta.
    const int kBrutos = 64;
    float cs[kBrutos];
    int cxs[kBrutos], cys[kBrutos], n = 0;
    const int Wm = z.width;
    const float *m = _mapa.data();
    for (int y = 1; y < z.height - 1; y++)
    {
        const float *f = m + (size_t)y * Wm;
        for (int x = 1; x < Wm - 1; x++)
        {
            const float v = f[x];
            // Se descarta rapido lo que esta bajo el umbral o no entraria en la lista.
            if (v < u * u || (n == kBrutos && v <= cs[kBrutos - 1]))
                continue;
            // f[x - Wm] es el vecino de arriba y f[x + Wm] el de abajo (una fila tiene Wm
            // elementos).
            if (v <= f[x - 1] || v <= f[x - Wm - 1] || v <= f[x - Wm] || v <= f[x - Wm + 1] ||
                v < f[x + 1] || v < f[x + Wm - 1] || v < f[x + Wm] || v < f[x + Wm + 1])
                continue;
            int i = n < kBrutos ? n++ : kBrutos - 1;
            while (i > 0 && cs[i - 1] < v)
            {
                cs[i] = cs[i - 1]; cxs[i] = cxs[i - 1]; cys[i] = cys[i - 1];
                i--;
            }
            cs[i] = v; cxs[i] = x + z.x; cys[i] = y + z.y;
        }
    }

    // Supresion espacial antes de cortar la lista: un objeto extendido con ruido (una
    // tarjeta de color, el borde de algo brillante) tiene decenas de maximos locales, todos
    // mas fuertes que un blanco chico, y sin esto ocupan todos los cupos. De mas fuerte a
    // mas debil, un candidato entra solo si esta a mas de una guarda de los que ya entraron.
    const int K = std::max(1, std::min(params.max_candidatos, 16));
    cv::Point2f cand[16];
    int nc = 0;
    const float r2 = (float)guarda_px * guarda_px;
    for (int i = 0; i < n && nc < K; i++)
    {
        // Coordenadas de trabajo -> pixeles de imagen (centro del bloque de d x d).
        const cv::Point2f c(t.origen.x + cxs[i] * d + 0.5f * (d - 1), t.origen.y + cys[i] * d + 0.5f * (d - 1));
        bool cerca = false;
        for (int j = 0; j < nc && !cerca; j++)
        {
            const cv::Point2f e = c - cand[j];
            cerca = e.dot(e) < r2;
        }
        if (excluir)
        {
            const cv::Point2f e = c - *excluir;
            cerca = cerca || e.dot(e) < radio_excl * radio_excl;
        }
        if (!cerca)
            cand[nc++] = c;
    }

    // En seguimiento se verifica primero lo mas cercano a la prediccion: un destello o
    // un reflejo dentro de la ventana puede tener mas snr que el blanco.
    if (pred)
    {
        const cv::Point2f q = *pred;
        std::sort(cand, cand + nc, [q](const cv::Point2f &a, const cv::Point2f &b) {
            const cv::Point2f da = a - q, db = b - q;
            return da.dot(da) < db.dot(db);
        });
    }

    // Verificacion fina: el primer candidato que pasa el CFAR a resolucion completa y la
    // prueba de forma es el elegido. La ventana fina es chica (d + 1 px de semilado),
    // porque el candidato ya viene ubicado con la precision del submuestreo.
    _rechazo = SNR_BAJO;
    for (int i = 0; i < nc; i++)
    {
        Deteccion det;
        const bool ok = detectarEnVentana(frame, cand[i], (float)(d + 1), guarda_px, umbral, det);
        if (ok && formaValida(det, v_lim))
        {
            if (!todas)
            {
                out = det;
                return true;
            }
            todas->push_back(det);
            continue;
        }
        if (ok)
            _rechazo = FORMA;
    }
    return todas && !todas->empty();
}
//---------------------------------------------------------------------------
// Misma guarda y umbral que la adquisicion de update(), sobre toda la imagen y sin pista.
int SpotTracker::candidatosGlobales(const cv::Mat &frame, cv::Point2f excluir, float radio_excl,
                                    std::vector<Deteccion> &out)
{
    out.clear();
    const float v_lim = params.vel_adquisicion;
    const int guarda = techo(params.radio_blob_max + 1.0f + 0.6f * v_lim);
    // detectarEnVentana() anota el motivo de rechazo y la ventana: se restauran.
    const Rechazo rechazo = _rechazo;
    const cv::Rect ventana = _ventana;
    Deteccion det;
    buscarGrueso(frame, cv::Rect(0, 0, frame.cols, frame.rows), guarda, params.umbral_adquisicion,
                 v_lim, nullptr, det, &out, &excluir, radio_excl);
    _rechazo = rechazo;
    _ventana = ventana;
    std::sort(out.begin(), out.end(), [](const Deteccion &a, const Deteccion &b) { return a.snr > b.snr; });
    return (int)out.size();
}
//---------------------------------------------------------------------------
// Un punto quieto de radio <= radio_blob_max tiene desviacion <= radio/sqrt(2): ese es
// el caso de un anillo (en los canales CROMA el nucleo saturado del laser queda blanco,
// o sea en cero, y solo se ve el halo); un disco lleno da radio/2. Moviendose deja una
// estela: sigue igual de angosta, pero se alarga hasta ~v px (una recta de largo L tiene
// desviacion L/sqrt(12)). Un objeto ancho, como una tarjeta de color o el borde de una
// lampara, no pasa aunque la guarda lo contenga.
//
// En la practica: el eje menor tiene que ser como maximo 0.7071 (= 1/raiz(2)) veces
// radio_blob_max, y el eje mayor puede ser eso mas 0.35 * v (aproximadamente v / raiz(12),
// que es lo que se alarga por la estela).
bool SpotTracker::formaValida(const Deteccion &det, float v) const
{
    const float r = 0.7071f * params.radio_blob_max;
    return det.eje_menor <= r && det.eje_mayor <= r + 0.35f * v;
}
//---------------------------------------------------------------------------
// Rapidez (px/cuadro) que dimensiona ventana y guarda. Se retiene el pico reciente en
// vez de usar |v| del alfa-beta: en un giro brusco o un rebote el filtro deja la
// velocidad cerca de cero justo cuando el blanco sigue yendo rapido.
float SpotTracker::rapidez() const
{
    return std::max(_rapidez, std::sqrt(_vel.x * _vel.x + _vel.y * _vel.y));
}
//---------------------------------------------------------------------------
// Largo de estela (px/cuadro) que admiten la guarda y la verificacion de forma. Nunca
// baja de vel_adquisicion: el blanco puede arrancar de golpe desde quieto, y ahi la
// rapidez medida todavia es cero. Admitir estela no deja pasar objetos anchos (eso lo
// filtra el eje menor) y agrandar la guarda no cuesta nada, por las integrales.
float SpotTracker::estela() const
{
    return std::max(rapidez(), params.vel_adquisicion);
}
//---------------------------------------------------------------------------
// La guarda tiene que contener al blob entero, incluida la estela que deja el
// movimiento durante la exposicion: crece con el tamano del ultimo blob y con la
// estela admitida.
// Igual se le pone un techo: nunca mas que lo que ocuparia el blob mas grande permitido
// (radio_blob_max) con esa estela.
int SpotTracker::guardaSeguimiento() const
{
    const float v = estela();
    const float g = std::max((float)params.guarda_min, 1.5f * _radio_blob + 1.0f) + 0.6f * v;
    return std::min(techo(g), techo(params.radio_blob_max + 1.0f + 0.6f * v));
}
//---------------------------------------------------------------------------
// Guarda una deteccion como la ultima aceptada. El radio del blob se estima como el de
// un circulo con la misma area (area = pi * r^2, asi que r = raiz(area / pi)).
void SpotTracker::aceptar(const Deteccion &det)
{
    _det = det;
    _detectado = true;
    _radio_blob = std::sqrt((float)det.area / (float)CV_PI);
}
//---------------------------------------------------------------------------
// Procesa un cuadro (la maquina de estados descrita en spottracker.hpp).
bool SpotTracker::update(const cv::Mat &frame)
{
    CV_Assert(frame.depth() == CV_8U && (frame.channels() == 1 || frame.channels() == 3));
    _detectado = false;
    // La sugerencia vale para este cuadro, se use o no.
    const bool sugerencia = _sugerencia;
    _sugerencia = false;

    // --- TENTATIVO o SIGUIENDO: busqueda en una ventana alrededor de la prediccion ---
    if (_estado != BUSCANDO)
    {
        // Prediccion del filtro alfa-beta: la posicion mas la velocidad (en px/cuadro).
        const cv::Point2f pred = _pos + _vel;
        const float v = rapidez();
        // En TENTATIVO todavia no hay velocidad confiable: ventana amplia.
        float radio = _estado == TENTATIVO ? 0.5f * params.radio_max
                                           : std::max(params.radio_min, params.k_vel * v);
        // Con cada cuadro fallido la ventana crece (crecimiento^fallos), hasta radio_max.
        radio = std::min(radio * std::pow(params.crecimiento, (float)_fallos), params.radio_max);
        const float umbral = _estado == SIGUIENDO ? params.umbral_seguimiento
                                                  : params.umbral_adquisicion;
        const int guarda = guardaSeguimiento();

        Deteccion det;
        const int r = techo(radio);
        const cv::Rect ventana((int)std::floor(pred.x + 0.5f) - r, (int)std::floor(pred.y + 0.5f) - r,
                               2 * r + 1, 2 * r + 1);
        bool ok;
        // Ventana grande: busqueda gruesa a fina. Ventana chica: CFAR directo a resolucion
        // completa, mas la verificacion de forma.
        if (radio > params.radio_grueso && params.decimacion > 1)
            ok = buscarGrueso(frame, ventana, guarda, umbral, estela(), &pred, det);
        else
        {
            ok = detectarEnVentana(frame, pred, radio, guarda, umbral, det);
            if (ok && !formaValida(det, estela()))
            {
                ok = false;
                _rechazo = FORMA;
            }
        }
        _ventana = ventana & cv::Rect(0, 0, frame.cols, frame.rows);

        if (ok)
        {
            if (_aciertos == 1)
            {
                // Arranque de dos puntos: con la primera y la segunda deteccion ya se
                // puede calcular una velocidad (la diferencia de posiciones).
                _vel = det.centro - _pos;   // arranque de dos puntos
                _pos = det.centro;
            }
            else
            {
                // Filtro alfa-beta: "res" es el error de la prediccion. La posicion se
                // corrige una fraccion alfa de ese error y la velocidad una fraccion beta.
                const cv::Point2f res = det.centro - pred;
                _pos = pred + params.alfa * res;
                _vel += params.beta * res;
            }
            // Rapidez retenida: el maximo entre la rapidez actual y la anterior "gastada"
            // un poco (multiplicada por retencion_rapidez).
            _rapidez = std::max(std::sqrt(_vel.x * _vel.x + _vel.y * _vel.y),
                                params.retencion_rapidez * _rapidez);
            _aciertos++;
            _fallos = 0;
            if (_estado == TENTATIVO && _aciertos >= params.confirmaciones)
                _estado = SIGUIENDO;
            aceptar(det);
            return true;
        }

        _pos = pred;   // se sigue por inercia
        _fallos++;
        // En SIGUIENDO se toleran hasta max_fallos cuadros sin deteccion. Un candidato en
        // TENTATIVO que falla una sola vez se descarta de inmediato.
        if (_estado == SIGUIENDO && _fallos <= params.max_fallos)
            return false;
        reset();       // y en el mismo cuadro se busca en toda la imagen
    }

    // --- BUSCANDO: adquisicion ---
    // Adquisicion: el blanco puede venir ya en movimiento, se admite una estela de hasta
    // vel_adquisicion px por cuadro (la guarda la tiene que contener).
    Deteccion det;
    const float v_lim = params.vel_adquisicion;
    const int guarda = techo(params.radio_blob_max + 1.0f + 0.6f * v_lim);
    const cv::Rect imagen(0, 0, frame.cols, frame.rows);
    bool ok = false;
    if (sugerencia)
    {
        // Primero cerca de la pista; si la zona ya es media imagen, no vale la pena.
        const int r = techo(_sug_radio);
        const cv::Rect zona = cv::Rect((int)std::floor(_sug_centro.x + 0.5f) - r,
                                       (int)std::floor(_sug_centro.y + 0.5f) - r, 2 * r + 1,
                                       2 * r + 1) & imagen;
        if (zona.area() > 0 && zona.area() < imagen.area() / 2)
            ok = buscarGrueso(frame, zona, guarda, params.umbral_adquisicion, v_lim, &_sug_centro, det);
    }
    // Si no se encontro cerca de la pista (o no habia pista), se busca en toda la imagen.
    if (!ok && params.ambiguedad <= 0.0f)
        ok = buscarGrueso(frame, imagen, guarda, params.umbral_adquisicion, v_lim,
                          sugerencia ? &_sug_centro : nullptr, det);
    else if (!ok)
    {
        // Con la regla de ambiguedad se verifican todos los candidatos: el elegido es el
        // primero (el mismo que sin la regla) y ninguno lejano puede acercarsele en snr.
        _candidatos.clear();
        ok = buscarGrueso(frame, imagen, guarda, params.umbral_adquisicion, v_lim,
                          sugerencia ? &_sug_centro : nullptr, det, &_candidatos);
        if (ok)
        {
            det = _candidatos[0];
            const float sep = 2.0f * params.radio_blob_max;
            for (size_t i = 1; i < _candidatos.size() && ok; i++)
            {
                const cv::Point2f e = _candidatos[i].centro - det.centro;
                if (e.dot(e) > sep * sep && _candidatos[i].snr >= params.ambiguedad * det.snr)
                {
                    _rechazo = AMBIGUO;
                    ok = false;
                }
            }
        }
    }
    _ventana = cv::Rect();
    if (!ok)
        return false;
    // Candidato encontrado: pasa a TENTATIVO (o directo a SIGUIENDO si no se piden
    // confirmaciones), quieto hasta que la segunda deteccion permita medir su velocidad.
    _estado = params.confirmaciones <= 1 ? SIGUIENDO : TENTATIVO;
    _pos = det.centro;
    _vel = cv::Point2f(0.0f, 0.0f);
    _rapidez = 0.0f;
    _aciertos = 1;
    _fallos = 0;
    aceptar(det);
    return true;
}
//---------------------------------------------------------------------------
// Siembra desde afuera: busca el blanco en una ventana del tamano de roi, con una guarda
// suficiente para contenerlo, y verifica su forma.
bool SpotTracker::init(const cv::Rect &roi, const cv::Mat &frame)
{
    reset();
    const cv::Point2f c(roi.x + 0.5f * roi.width, roi.y + 0.5f * roi.height);
    const int lado = std::max(roi.width, roi.height);
    Deteccion det;
    if (!detectarEnVentana(frame, c, 0.5f * lado, std::max(params.guarda_min, lado / 2 + 1),
                           params.umbral_adquisicion, det) ||
        !formaValida(det, 0.5f * lado))
        return false;
    // La confirmacion la dio el detector externo: se pasa directo a SIGUIENDO.
    _estado = SIGUIENDO;
    _pos = det.centro;
    _aciertos = 1;
    aceptar(det);
    return true;
}

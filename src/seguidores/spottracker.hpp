// Detector y seguidor de blancos puntuales de alto contraste (un puntero laser sobre un
// fondo claro, un LED, un reflector), pensado para el mismo lazo pan/tilt que el
// KCFTracker, pero con una gran diferencia: no necesita que alguien le marque una ventana
// inicial, sino que encuentra el blanco por si solo.
//
// DETECCION
//
// Se usa un detector CA-CFAR ("cell-averaging constant false alarm rate", un nombre que
// viene del mundo de los radares) sobre un canal escalar de la imagen (por ejemplo, el
// brillo). La idea es muy intuitiva: un pixel es "interesante" si es mucho mas brillante
// que lo que tiene alrededor. Para cada pixel se definen tres zonas concentricas:
//
//         +-------------------+
//         |      anillo       |
//         |   +-----------+   |
//         |   |  guarda   |   |
//         |   |   [CUT]   |   |
//         |   +-----------+   |
//         +-------------------+
//
// En el centro esta la celda bajo prueba (CUT, "cell under test"), que es el pixel que
// se esta evaluando (o un cuadradito de 3x3 a su alrededor, para promediar el ruido). Por
// fuera hay un anillo de "entrenamiento", del que se calcula la media mu y la desviacion
// sigma del fondo local. Entre ambos hay una zona de "guarda" que no se usa para nada: su
// funcion es que la luz del propio blanco (que se desparrama un poco) no contamine la
// estimacion del fondo. Con eso se calcula
//     snr = (media(CUT) - mu) / max(sigma, sigma_min)
// que dice cuantas "desviaciones del fondo" sobresale el pixel, y hay deteccion si el snr
// supera un umbral.
//
// Este metodo tiene dos ventajas importantes:
//  - Iluminacion: el resultado no cambia si la imagen entera se multiplica por un numero y
//    se le suma otro (I -> a*I + b, que es lo que hacen la ganancia y el offset de la
//    camara, o una sombra suave), porque se compara contra el fondo local. Ademas, no hay
//    un modelo de apariencia que "envejezca", a diferencia del KCF.
//  - Costo: con imagenes integrales (ver spottracker.cpp), sumar cualquier ventana cuesta
//    4 lecturas de memoria, sin importar su tamano. Por eso la guarda puede crecer con la
//    velocidad del blanco (para contener su estela de movimiento) sin encarecer nada. Un
//    filtro "top-hat" morfologico haria algo parecido a restar mu, pero cuesta mas cuanto
//    mas grande es su elemento, y ademas necesitaria un umbral aparte.
//
// SEGUIMIENTO
//
// El seguidor es una maquina de tres estados.
//
// En BUSCANDO, se corre el CFAR sobre la imagen completa, submuestreada para que sea
// rapido. Los mejores maximos locales se verifican luego a resolucion completa: se mide la
// mancha (componente conexa) del pico, su centroide con precision subpixel y su forma. La
// forma tiene que ser angosta: una estela de movimiento puede ser larga, pero no ancha.
//
// En TENTATIVO, el candidato tiene que repetirse `confirmaciones` cuadros seguidos antes
// de aceptarse. Esto filtra destellos que duran un solo cuadro.
//
// En SIGUIENDO, el CFAR se corre solo en una ventana alrededor de la posicion predicha
// (con un filtro alfa-beta). La ventana crece con la rapidez del blanco y con cada cuadro
// en que no se lo encuentra; si se vuelve grande, se barre submuestreada. Tras
// `max_fallos` cuadros seguidos sin deteccion, se vuelve a BUSCANDO.
//
// La medicion que entrega (el centroide subpixel) es la que consume el filtro de Kalman del
// control; la prediccion alfa-beta de aca solo sirve para ubicar la ventana de busqueda.
//
// Uso tipico:
//
//     SpotTracker tracker;                       // o SpotTracker(params)
//     for (;;) {
//         tracker.shift(-pan_px, -tilt_px);      // opcional: movimiento propio de la camara
//         tracker.update(frame);                 // frame: 8 bits, BGR o gris
//         cv::Point2f z;
//         if (tracker.medicion(z))
//             ;   // z va a kalman_actualizar(); si no, se le pasa NULL
//     }

#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>

class SpotTracker
{
public:
    // Canal escalar sobre el que corre el CFAR. Los CROMA_* restan el maximo de los otros
    // dos canales: un fondo blanco o gris da ~0 aunque este saturado, y el halo de un laser
    // rojo da mucho. Conviene para laseres de color con la exposicion baja.
    enum Canal { GRIS, ROJO, VERDE, AZUL, CROMA_ROJO, CROMA_VERDE, CROMA_AZUL };
    enum Estado { BUSCANDO, TENTATIVO, SIGUIENDO };
    // Por que no hubo deteccion en el ultimo intento (para calibrar sobre metraje propio):
    // no hubo motivo (NINGUNO), la ventana quedo fuera de la imagen, el snr fue muy bajo,
    // la mancha era demasiado extensa (se salia de la guarda), tenia mala forma o habia
    // otro candidato casi tan bueno (AMBIGUO).
    enum Rechazo { NINGUNO, FUERA_DE_IMAGEN, SNR_BAJO, EXTENDIDO, FORMA, AMBIGUO };

    struct Params
    {
        Canal canal;
        bool oscuro;              // blanco mas oscuro que el fondo: se invierte el canal

        // --- CFAR ---------------------------------------------------------------
        float umbral_adquisicion; // snr minimo para adquirir en la imagen completa
        // snr minimo dentro de la ventana de seguimiento. Es menor que el de adquisicion
        // (histeresis): cuesta mas engancharse que mantenerse enganchado.
        float umbral_seguimiento;
        // Piso de sigma, en niveles de gris. Un fondo saturado (todo en 255) tiene
        // sigma ~0, y sin este piso cualquier ruido daria un snr enorme.
        float sigma_min;
        // Altura minima del pico sobre el fondo, en niveles del canal (0..255). El snr solo
        // no basta: en croma el fondo oscuro tiene sigma ~0 y una mancha de pocos niveles
        // alcanza el umbral contra el piso sigma_min. 0 = sin este criterio.
        float contraste_min;
        int celda;                // semilado de la CUT (px): 1 -> 3x3, promedia ruido
        int guarda_min;           // semilado minimo de la guarda (px)
        // Espesor del anillo de entrenamiento (px, como maximo 10). Guarda + anillo se
        // acota a 90 px de trabajo (ver kSemiladoMax en spottracker.cpp).
        int anillo;
        // Radio maximo del blanco quieto (px). Lo que sea mas ancho (una lampara, una
        // tarjeta de color, el borde de algo brillante) se descarta. En movimiento se
        // admite que se alargue con la estela, pero no que engorde.
        float radio_blob_max;

        // --- busqueda -----------------------------------------------------------
        int decimacion;           // submuestreo de la busqueda global (1 = sin submuestreo)
        int max_candidatos;       // maximos locales que se verifican por cuadro en BUSCANDO
        // Estela admitida al adquirir, y minima en seguimiento (px/cuadro). Un valor mas
        // grande aleja el anillo del fondo.
        float vel_adquisicion;
        float radio_min;          // semilado minimo de la ventana de seguimiento (px)
        float radio_max;          // semilado maximo (px)
        float k_vel;              // la ventana cubre k_vel * |v| (v en px/cuadro)
        float crecimiento;        // factor de crecimiento de la ventana por cuadro perdido
        float retencion_rapidez;  // decaimiento por cuadro del pico de rapidez retenido
        // Sobre este semilado, la ventana se barre submuestreada (de grueso a fino). Un
        // blanco tan rapido deja una estela larga, asi que no se pierde nada.
        float radio_grueso;
        int confirmaciones;       // detecciones seguidas para pasar de TENTATIVO a SIGUIENDO
        // Al adquirir en la imagen completa, si otro candidato (a mas de 2 radio_blob_max)
        // tiene snr >= ambiguedad * el del elegido, no se adquiere ninguno: es preferible
        // no medir a afirmar un blanco dudoso. 0 = sin esta regla.
        float ambiguedad;
        int max_fallos;           // cuadros perdidos seguidos antes de volver a BUSCANDO

        // --- filtro alfa-beta de la ventana ------------------------------------
        // alfa: cuanto se corrige la posicion hacia la medicion (0 = nada, 1 = todo).
        // beta: cuanto se corrige la velocidad.
        float alfa, beta;

        Params();
    };

    // Resultado de una deteccion.
    struct Deteccion
    {
        cv::Point2f centro;       // centroide subpixel (px, imagen completa)
        cv::Rect caja;            // caja de la componente conexa del pico
        float snr;                // snr CFAR del pico
        int area;                 // pixeles de esa componente
        // Desviaciones de la elipse equivalente (px), a lo largo de su eje mayor y de su
        // eje menor. Una estela de movimiento es larga y angosta (eje_mayor grande y
        // eje_menor chico).
        float eje_mayor;
        float eje_menor;
    };

    explicit SpotTracker(const Params &p = Params());

    // Procesa un cuadro. Devuelve true si hubo deteccion en este cuadro (en cualquier
    // estado; para el lazo de control usar medicion()).
    bool update(const cv::Mat &frame);

    // Medicion para el estimador: solo en SIGUIENDO y si hubo deteccion en este cuadro.
    bool medicion(cv::Point2f &z) const;

    // Sembrar el seguimiento desde afuera (un adquisidor externo): busca el blanco
    // dentro de `roi` y, si lo encuentra, pasa directo a SIGUIENDO (la confirmacion la
    // dio el detector externo). Devuelve si lo encontro.
    bool init(const cv::Rect &roi, const cv::Mat &frame);

    // Vuelve al estado inicial (BUSCANDO, sin historia).
    void reset();

    // Pista externa para la busqueda global (la prediccion del Kalman, por ejemplo). Vale
    // solo para el proximo update(): si en ese cuadro hay que buscar en toda la imagen,
    // primero se busca en un cuadrado de semilado `radio` alrededor de `centro`, y en
    // cualquier caso los candidatos se prueban por cercania a `centro` y no por snr.
    void sugerir(cv::Point2f centro, float radio)
    {
        _sugerencia = true;
        _sug_centro = centro;
        _sug_radio = radio;
    }

    // Busqueda de adquisicion en toda la imagen que no toca el estado del seguidor (la usa la
    // barrida global, retador.h): devuelve en out todas las detecciones verificadas (hasta
    // max_candidatos), de mayor a menor snr, salvo las que esten a menos de radio_excl de
    // excluir. Devuelve cuantas encontro.
    int candidatosGlobales(const cv::Mat &frame, cv::Point2f excluir, float radio_excl,
                           std::vector<Deteccion> &out);

    // Trasladar la prediccion antes del proximo update() (movimiento de la torreta).
    void shift(float dx, float dy) { _pos.x += dx; _pos.y += dy; }

    bool lost() const { return _estado != SIGUIENDO; }
    Estado estado() const { return _estado; }
    const Deteccion &ultima() const { return _det; }     // valida si detectado()
    bool detectado() const { return _detectado; }
    cv::Point2f posicion() const { return _pos; }        // estimada (alfa-beta)
    cv::Point2f velocidad() const { return _vel; }       // px/cuadro
    float snr() const { return _detectado ? _det.snr : 0.0f; }
    // Ventana de busqueda usada en el ultimo update (vacia si fue busqueda global).
    cv::Rect ventana() const { return _ventana; }
    Rechazo rechazo() const { return _rechazo; }

    Params params;

private:
    // Region de trabajo: el canal extraido (quizas submuestreado) y sus imagenes integrales.
    // "Pixeles de trabajo" son los pixeles de esta region: si d = 2, cada pixel de trabajo
    // es el promedio de un bloque de 2x2 pixeles de la imagen.
    struct Trabajo
    {
        int W, H, d;             // tamano en pixeles de trabajo y factor de submuestreo
        cv::Point origen;        // esquina de la region en la imagen completa
        std::vector<uint8_t> ch; // canal, W*H
        std::vector<uint32_t> S; // integral de ch, (W+1)*(H+1), modulo 2^32
        std::vector<uint32_t> Q; // integral de ch^2, modulo 2^32
    };

    void extraer(const cv::Mat &frame, const cv::Rect &r, int d, Trabajo &t) const;
    bool detectarEnVentana(const cv::Mat &frame, cv::Point2f centro, float radio, int guarda,
                           float umbral, Deteccion &out);
    // todas: si no es nulo, no se detiene en el primer candidato valido y guarda todos los
    // que pasan (salvo los que esten a menos de radio_excl de *excluir).
    bool buscarGrueso(const cv::Mat &frame, const cv::Rect &zona, int guarda_px, float umbral,
                      float v_lim, const cv::Point2f *pred, Deteccion &out,
                      std::vector<Deteccion> *todas = nullptr, const cv::Point2f *excluir = nullptr,
                      float radio_excl = 0.0f);
    bool formaValida(const Deteccion &det, float v) const;
    float rapidez() const;
    float estela() const;
    int guardaSeguimiento() const;
    void aceptar(const Deteccion &det);

    // Espacios de trabajo. Se reservan una vez y se reutilizan en cada cuadro, para no
    // pedir memoria nueva en cada llamada.
    Trabajo _local, _global;     // region de la ventana local y de la busqueda gruesa
    std::vector<float> _mapa;    // snr^2 de la busqueda gruesa
    std::vector<float> _fila;    // snr^2 de una fila, en la busqueda fina
    std::vector<uint8_t> _marca; // relleno de la componente conexa
    std::vector<int> _pila;      // pila de pixeles pendientes del relleno
    std::vector<Deteccion> _candidatos; // de la adquisicion global, para la regla de ambiguedad

    Estado _estado;
    cv::Point2f _pos, _vel;      // posicion y velocidad del filtro alfa-beta
    int _aciertos, _fallos;      // detecciones y fallos seguidos
    bool _detectado;             // hubo deteccion en el ultimo update
    Deteccion _det;              // ultima deteccion aceptada
    float _radio_blob;           // radio del ultimo blob, para adaptar la guarda
    float _rapidez;              // pico de |v| retenido (px/cuadro)
    cv::Rect _ventana;
    Rechazo _rechazo;
    bool _sugerencia;            // pista externa para la proxima busqueda global
    cv::Point2f _sug_centro;
    float _sug_radio;
};

// Seguidor por Filtro de Correlacion Kernelizado (KCF) con caracteristicas HOG de
// Felzenszwalb.
//
// Como funciona, en pocas palabras? Al comenzar, el seguidor recibe una caja con el
// blanco y "aprende" su apariencia: construye un filtro que, al compararlo con un pedazo
// de imagen, da una respuesta muy alta justo donde esta el blanco y baja en el resto. En
// cada cuadro nuevo, aplica ese filtro en una ventana alrededor de la ultima posicion
// conocida y mira donde quedo el pico de la respuesta: ese es el desplazamiento del
// blanco. Luego actualiza un poquito el filtro con la apariencia nueva, para adaptarse a
// cambios lentos (de luz, de angulo, etc.).
//
// El truco que lo hace tan rapido es que, para aprender, usa como ejemplos TODAS las
// versiones desplazadas del parche (corrido un pixel, dos, etc.), y gracias a una
// propiedad matematica de esos desplazamientos (forman una matriz "circulante"), todas
// las cuentas se pueden hacer con la transformada de Fourier (FFT) en muy poco tiempo. La
// parte "kernelizada" significa que la comparacion no es un simple producto, sino una
// funcion mas flexible (un kernel gaussiano), lo que le permite distinguir mejor al
// blanco del fondo.
//
// La notacion sigue la del articulo tutorial:
//
//   [1] G. Bhat et al., "Understanding the Kernelized Correlation Filter Tracker",
//       Technologies 2021, 9, 93.
//   [2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista, "High-Speed Tracking
//       with Kernelized Correlation Filters", TPAMI 2015.
//
// Para leer el codigo junto con el articulo, sirve saber como se llama aqui cada simbolo.
// El parche de entrenamiento x (la apariencia aprendida del blanco) es _x, y su DFT por
// canal x^ se guarda precalculada en _xf (ver train()). El parche candidato del cuadro
// nuevo, z, se llama igual. Los coeficientes duales en frecuencia alpha^ de la ec. (17)
// son _alphaf, y la DFT del objetivo de regresion gaussiano y^ de la ec. (1) es _yf. La
// primera fila de la matriz kernel (la autocorrelacion k^xx de la ec. (17)) es kxx, y la
// correlacion cruzada entre modelo y candidato k^xz de la ec. (18) es kxz. El mapa de
// respuesta f(z) de la ec. (18) es response. Por ultimo, lambda (el regularizador de la
// regresion ridge, ec. (1)) y sigma (el ancho de banda del kernel gaussiano, ec. (12))
// conservan su nombre.
//
// Puerto C++ original: Joao Faro, Christian Bailer, Joao F. Henriques
// (Institute of Systems and Robotics, Universidad de Coimbra / DFKI Augmented Vision).
// BSD 3-clause; ver LICENSE.
//
// Esta version conserva los anadidos del puerto que resultan utiles sobre una torreta
// pan/tilt (una medida de confianza PSR con congelamiento del modelo, limites duros a la
// deriva de escala, y un enganche para inyectar el movimiento propio de la camara) y
// corrige los defectos descritos en NOTAS.md.
//
// Uso tipico:
//
//     KCFTracker tracker;
//     tracker.init(roi, frame);              // frame: 8 bits, 1 o 3 canales
//
//     for (;;) {
//         // Opcional: decirle al seguidor cuanto se habra desplazado la escena por el
//         // comando que acabamos de enviar a los servos, para que el blanco siga dentro
//         // de la ventana de busqueda.
//         tracker.shift(-pan_px, -tilt_px);
//
//         cv::Rect box = tracker.update(frame);
//         if (tracker.lost())
//             ;   // sostener los servos, no perseguir ruido; el modelo esta congelado
//     }

#pragma once

#include <opencv2/core.hpp>
#include <vector>

class KCFTracker
{
public:
    KCFTracker();

    // Comenzar a seguir `roi` en `image`. `image` es de 8 bits, 1 o 3 canales.
    void init(const cv::Rect &roi, const cv::Mat &image);

    // Seguir hacia el cuadro siguiente. Siempre devuelve una caja; revisar lost()
    // antes de confiar en ella.
    cv::Rect update(const cv::Mat &image);

    // --- confianza ------------------------------------------------------------------
    // Razon pico a lobulo lateral (PSR, "peak to sidelobe ratio") del ultimo mapa de
    // respuesta f(z): cuanto sobresale el pico respecto del resto del mapa, en
    // desviaciones estandar. Un enganche solido suele estar bastante por encima de 10 y
    // se desploma cuando el blanco desaparece.
    float psr() const { return _psr; }

    // Referencia con la que se compara psr(): promedio exponencial del PSR sobre los
    // cuadros que el seguidor considero confiables. Sirve para elegir psr_drop_ratio
    // observandola sobre metraje propio.
    float psrReference() const { return _psr_ref; }

    // True cuando el mapa de respuesta deja de ser concluyente, sea porque psr() cae
    // bajo psr_threshold (piso absoluto) o porque cae bajo psr_drop_ratio veces
    // psrReference() (criterio relativo, independiente de la escena).
    //
    // Mientras esta perdido, update() deja intactos la caja, la escala y el filtro: el
    // modelo no puede "envenenarse" aprendiendo el fondo, y la ventana de busqueda queda
    // anclada en la ultima posicion confiable. Si el blanco reaparece cerca de ahi, el
    // seguidor vuelve a engancharse solo. Si el blanco se movio mucho durante la
    // oclusion, hay que reposicionar la ventana desde afuera con shift() o setROI(): esa
    // es la tarea de la capa de estimacion, no del seguidor.
    bool lost() const { return _lost; }

    // --- enganche pan/tilt ----------------------------------------------------------
    // Trasladar la ventana de busqueda, aplicado antes del proximo update(). KCF solo
    // busca en una ventana centrada en la posicion previa, de modo que un giro rapido
    // puede sacar al blanco de ella incluso con relleno generoso.
    void shift(float dx, float dy) { _roi.x += dx; _roi.y += dy; }

    // Caja actual, y forma de imponerla (readquisicion desde un detector externo).
    cv::Rect_<float> roi() const { return _roi; }
    void setROI(const cv::Rect_<float> &roi) { _roi = roi; }

    // --- parametros (fijar antes de init()) -----------------------------------------
    float interp_factor;       // tasa de aprendizaje del modelo; menor = mas lento y robusto
    float sigma;               // sigma: ancho de banda del kernel gaussiano
    float lambda;              // lambda: regularizador de la regresion ridge
    float padding;             // tamano de la ventana de busqueda respecto del blanco
    float output_sigma_factor; // ancho de banda del objetivo de regresion gaussiano
    int   cell_size;           // tamano de celda HOG, en pixeles
    int   template_size;       // lado mayor de la plantilla, en pixeles; menor = mas rapido
    float scale_step;          // paso multiescala; 1.0f desactiva la estimacion de escala

    // Penalizacion a un cambio de escala; menor que 1 hace que la escala sea mas
    // "pegajosa" (que cueste mas cambiarla). Se ajusta junto con scale_step: un cambio de
    // escala solo ocurre si mejora el pico en mas de 1/scale_weight, asi que un paso mas
    // fino necesita una penalizacion mas debil, o la escala se congela del todo.
    float scale_weight;

    // Limites duros al tamano del blanco respecto del que tenia en init(), para que la
    // estimacion de escala no derive hasta colapsar (achicarse a nada o crecer sin fin).
    float min_scale;
    float max_scale;

    float psr_threshold;       // piso absoluto de psr(); bajo esto el blanco cuenta como perdido

    // Criterio relativo: perdido si psr() < ratio * psrReference(). Es el criterio que de
    // verdad detecta oclusiones, porque la escala absoluta del PSR depende de la escena.
    // 0 lo desactiva. Sobre las secuencias de prueba, la razon psr()/psrReference() nunca
    // bajo de 0.37 con el blanco enganchado ni supero 0.27 con el blanco ocluido; 0.32
    // queda en medio de esa brecha. Para calibrarlo sobre metraje propio, conviene
    // registrar psr() y psrReference() y buscar la misma separacion. Es mejor errar hacia
    // declarar perdido: una falsa alarma cuesta un update omitido, mientras que una
    // oclusion no detectada envenena el modelo de forma permanente.
    float psr_drop_ratio;

private:
    // Resultado de buscar el blanco en un parche.
    struct Response
    {
        cv::Point2f shift; // desplazamiento del pico respecto del centro, en celdas HOG
        float peak;        // pico de correlacion en bruto
        float psr;         // razon pico a lobulo lateral
    };

    // Numero de cuadros confiables antes de que el criterio relativo entre en vigor
    // (antes de eso, la referencia todavia no es representativa).
    static const int kPsrWarmup = 12;
    // Tasa del promedio exponencial de psrReference().
    static constexpr float kPsrRefRate = 0.05f;

    Response detect(const cv::Mat &z) const;
    void train(const cv::Mat &x, float learning_rate);

    // DFT por canal de una matriz de caracteristicas (una fila por canal).
    void spectrum(const cv::Mat &x, std::vector<cv::Mat> &xf) const;

    // Kernel gaussiano entre dos parches dados por su espectro y su norma cuadrada,
    // para todos los desplazamientos ciclicos. Convencion del articulo: el
    // desplazamiento nulo queda en el indice (0,0), sin recentrar.
    cv::Mat kernelCorrelation(const std::vector<cv::Mat> &x1f, double x1_sqnorm,
                              const std::vector<cv::Mat> &x2f, double x2_sqnorm) const;

    cv::Mat createGaussianTarget(int rows, int cols) const;
    cv::Mat getFeatures(const cv::Mat &image, bool initHann, float scale_adjust = 1.0f);
    void createHanningMats();
    void clampROI(const cv::Mat &imagen);
    bool verificarEscalado(float ajuste) const;

    cv::Mat _alphaf;              // alpha^: coeficientes duales, ec. (17)
    cv::Mat _yf;                  // y^: objetivo de regresion gaussiano en frecuencia
    cv::Mat _x;                   // x: apariencia aprendida, una fila por canal
    std::vector<cv::Mat> _xf;     // x^: DFT de _x por canal, en cache
    double _x_sqnorm;             // ||x||^2, en cache
    cv::Mat _hann;                // ventana coseno, una fila por canal

    cv::Rect_<float> _roi;        // caja actual del blanco (px de imagen)
    cv::Size _tmpl_sz;            // tamano de la plantilla (px de plantilla)
    cv::Size _cells;   // tamano de la rejilla de caracteristicas, en celdas
    int _channels;     // canales de caracteristicas (31 para HOG de Felzenszwalb)
    float _scale;      // pixeles de imagen por pixel de plantilla
    float _init_size;  // sqrt(w*h) del roi inicial, referencia del limite de escala
    float _psr;        // PSR del ultimo update
    float _psr_ref;    // referencia del PSR (promedio exponencial)
    int _psr_ref_n;    // cuantos cuadros confiables lleva la referencia (hasta kPsrWarmup)
    bool _lost;
};

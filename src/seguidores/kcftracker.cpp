// Seguidor por Filtro de Correlacion Kernelizado (KCF) con caracteristicas HOG de
// Felzenszwalb. Ver kcftracker.hpp para una explicacion general del metodo y para la
// correspondencia entre los simbolos del articulo y los nombres de esta clase.
//
// Puerto C++ original: Joao Faro, Christian Bailer, Joao F. Henriques
// (Institute of Systems and Robotics, Universidad de Coimbra / DFKI Augmented Vision).
// BSD 3-clause; ver LICENSE.
//
// Una nota sobre como guarda OpenCV los numeros complejos: una matriz compleja es una
// cv::Mat de 2 canales, donde el canal 0 es la parte real y el canal 1 la imaginaria.
// Todas las transformadas de Fourier (cv::dft) de este archivo trabajan con ese formato.

#include "kcftracker.hpp"
#include "fhog.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

// Funciones auxiliares, visibles solo dentro de este archivo.
namespace {

// --- ayudantes de frecuencia --------------------------------------------------------

// Division elemento a elemento a / b sobre matrices complejas de 2 canales. Se usa la
// formula de la division de complejos: a / b = a * conj(b) / |b|^2. Separando parte real
// e imaginaria, (ar + i ai) / (br + i bi) = [(ar br + ai bi) + i (ai br - ar bi)] / (br^2 + bi^2).
cv::Mat complexDivision(const cv::Mat &a, const cv::Mat &b)
{
    // split separa cada matriz en sus dos canales: [0] = real, [1] = imaginaria.
    cv::Mat pa[2], pb[2];
    cv::split(a, pa);
    cv::split(b, pb);

    const cv::Mat divisor = 1.0 / (pb[0].mul(pb[0]) + pb[1].mul(pb[1])); // 1 / |b|^2

    cv::Mat pres[2];
    pres[0] = (pa[0].mul(pb[0]) + pa[1].mul(pb[1])).mul(divisor);
    pres[1] = (pa[1].mul(pb[0]) - pa[0].mul(pb[1])).mul(divisor);

    cv::Mat res;
    cv::merge(pres, 2, res); // vuelve a juntar los dos canales en una matriz compleja
    return res;
}
//---------------------------------------------------------------------------
// Llevar la muestra de desplazamiento nulo desde el indice (0,0) hasta el centro del
// mapa, para poder leer el pico como un desplazamiento con signo. Se aplica una sola
// vez, al mapa de respuesta f(z), y no al kernel: recentrar el kernel es justamente lo
// que rompia la regularizacion. Vale tambien para tamanos impares.
//
// Por que hace falta: en el mapa que sale de la FFT, "sin desplazamiento" esta en la
// esquina (0,0), un desplazamiento de +1 esta en la posicion 1, y uno de -1 esta al otro
// extremo (en la ultima posicion), porque todo es ciclico. Al correr el mapa media
// vuelta en cada eje, el "sin desplazamiento" queda al medio, los positivos a su derecha
// y los negativos a su izquierda, que es mucho mas facil de interpretar.
cv::Mat fftShift(const cv::Mat &src)
{
    CV_Assert(src.type() == CV_32F);

    cv::Mat dst(src.size(), src.type());
    const int rows = src.rows, cols = src.cols;
    const int sy = rows / 2, sx = cols / 2; // media vuelta en cada eje

    // Cada elemento (i, j) se copia a (i + sy, j + sx), dando la vuelta con % si se pasa.
    for (int i = 0; i < rows; i++)
    {
        const float *s = src.ptr<float>(i);
        float *d = dst.ptr<float>((i + sy) % rows);
        for (int j = 0; j < cols; j++)
            d[(j + sx) % cols] = s[j];
    }
    return dst;
}

// --- ayudantes varios ---------------------------------------------------------------

// Recortar `window` de `in`, rellenando lo que caiga fuera de la imagen. A diferencia
// del original, esto nunca hace assert: la ventana puede quedar parcial o totalmente
// fuera, cosa que ocurre de forma rutinaria mientras la torreta gira. El parche
// devuelto puede ser mas chico que el pedido en el caso degenerado; el llamador lo
// redimensiona de todos modos.
cv::Mat subwindow(const cv::Mat &in, const cv::Rect &window, int borderType)
{
    // Parte de la ventana que si esta dentro de la imagen.
    cv::Rect inside = window & cv::Rect(0, 0, in.cols, in.rows);

    // Si la ventana quedo completamente afuera, se toma el pixel de la imagen mas cercano
    // a ella (un parche de 1x1) y el relleno hace el resto.
    if (inside.area() == 0)
    {
        inside.x = std::min(std::max(window.x, 0), in.cols - 1);
        inside.y = std::min(std::max(window.y, 0), in.rows - 1);
        inside.width = inside.height = 1;
    }

    cv::Mat res = in(inside);

    // Cuantos pixeles faltan por cada lado para completar la ventana pedida.
    const int top = std::max(0, inside.y - window.y);
    const int left = std::max(0, inside.x - window.x);
    const int bottom = std::max(0, window.br().y - inside.br().y);
    const int right = std::max(0, window.br().x - inside.br().x);
    // copyMakeBorder agrega ese relleno. Con BORDER_REPLICATE, el relleno repite el pixel
    // del borde (en vez de poner negro, que pareceria un borde falso al seguidor).
    if (top || left || bottom || right)
        cv::copyMakeBorder(res, res, top, bottom, left, right, borderType);

    return res;
}
//---------------------------------------------------------------------------
// Interpolacion parabolica por tres muestras, para localizar el pico bajo la celda.
// Con `center` siendo el maximo, el divisor es no negativo y el resultado queda acotado
// en +-0.5 celdas.
//
// La idea: el pico verdadero rara vez cae justo sobre una celda. Si se ajusta una
// parabola que pase por el maximo y sus dos vecinos, el vertice de esa parabola da una
// estimacion del pico con precision de fracciones de celda. La formula devuelve cuanto
// hay que correrse desde la celda del medio (negativo = hacia la izquierda).
float subPixelPeak(float left, float center, float right)
{
    const float divisor = 2 * center - right - left;
    if (divisor == 0)
        return 0;
    return 0.5f * (right - left) / divisor;
}
//---------------------------------------------------------------------------
// Razon pico a lobulo lateral: (pico - media) / desviacion sobre el mapa de respuesta,
// excluyendo una ventana chica alrededor del pico. La exclusion se hace con indices
// ciclicos, porque el mapa es ciclico y el pico puede caer junto a un borde.
//
// Se excluye la zona del pico porque ahi la respuesta es alta "por definicion" (es la
// misma montana del pico). Lo que interesa es comparar el pico contra el resto del mapa:
// si el resto es plano y bajo, el pico sobresale mucho y el PSR es alto (el seguidor esta
// seguro); si hay otras montanas parecidas, el PSR es bajo (hay confusion).
float computePSR(const cv::Mat &response, cv::Point peak, float peak_value)
{
    const int rows = response.rows, cols = response.cols;

    // Radio de exclusion de 5 como en el articulo de MOSSE, reducido si el mapa es chico.
    const int radius = std::max(1, std::min(5, std::min(cols, rows) / 4));

    // Mascara: 255 = se usa para la estadistica, 0 = se excluye. El truco
    // ((a % n) + n) % n da el resto siempre positivo, incluso si a es negativo, lo que
    // permite "dar la vuelta" por los bordes.
    cv::Mat mask(response.size(), CV_8U, cv::Scalar(255));
    for (int di = -radius; di <= radius; di++)
    {
        uchar *m = mask.ptr<uchar>(((peak.y + di) % rows + rows) % rows);
        for (int dj = -radius; dj <= radius; dj++)
            m[((peak.x + dj) % cols + cols) % cols] = 0;
    }

    cv::Scalar mean, stddev;
    cv::meanStdDev(response, mean, stddev, mask);
    if (stddev[0] < 1e-9)
        return 0.0f; // mapa completamente plano: no hay pico que sobresalga

    return (peak_value - (float)mean[0]) / (float)stddev[0];
}

} // namespace
// =====================================================================
// Constructor: solo fija los valores por defecto de los parametros. El seguimiento
// comienza de verdad al llamar a init().
KCFTracker::KCFTracker()
    : interp_factor(0.012f),
      sigma(0.6f),
      lambda(0.0001f),
      padding(2.5f),
      output_sigma_factor(0.125f),
      cell_size(4),
      template_size(96),
      scale_step(1.02f),
      scale_weight(0.98f),
      min_scale(0.4f),
      max_scale(2.5f),
      psr_threshold(5.0f),
      psr_drop_ratio(0.32f),
      _x_sqnorm(0.0),
      _tmpl_sz(0, 0),
      _cells(0, 0),
      _channels(0),
      _scale(1.0f),
      _init_size(1.0f),
      _psr(0.0f),
      _psr_ref(0.0f),
      _psr_ref_n(0),
      _lost(false)
{
}
// -----------------------------------------------------------------------
// Comienza a seguir el blanco que esta en `roi`.
void KCFTracker::init(const cv::Rect &roi, const cv::Mat &image)
{
    _roi = roi;

    // Un roi degenerado colapsaria a cero la escala de la plantilla.
    // Por eso se le exige un tamano minimo de dos celdas HOG por lado.
    _roi.width = std::max(_roi.width, (float)(2 * cell_size));
    _roi.height = std::max(_roi.height, (float)(2 * cell_size));

    _init_size = std::sqrt(_roi.width * _roi.height);
    _psr = 0.0f;
    _psr_ref = 0.0f;
    _psr_ref_n = 0;
    _lost = false;

    // Se extraen las caracteristicas del parche inicial (y, como es la primera vez, se
    // calculan tambien el tamano de la plantilla y la ventana coseno), se crea el
    // objetivo gaussiano y se entrena el modelo desde cero (learning_rate = 1 significa
    // "olvidar todo lo anterior y quedarse solo con esto").
    const cv::Mat x = getFeatures(image, true);
    _yf = createGaussianTarget(_cells.height, _cells.width);
    _alphaf = cv::Mat::zeros(_cells.height, _cells.width, CV_32FC2);

    train(x, 1.0f);
}
// -----------------------------------------------------------------------
// Funcion para mantener el centro de la ROI dentro del frame.
void KCFTracker::clampROI(const cv::Mat &imagen)
{
    // Calculo del centro de la ROI:
    float cx = _roi.x + _roi.width / 2.0f;
    float cy = _roi.y + _roi.height / 2.0f;

    // Reposicionamiento de la ROI para que permanezca en el frame:
    cx = std::min(std::max(cx, 0.0f), imagen.cols - 1.0f);
    cy = std::min(std::max(cy, 0.0f), imagen.rows - 1.0f);

    // Calculo de la esquina superior izquierda de la ROI en base al centro.
    _roi.x = cx - _roi.width / 2.0f;
    _roi.y = cy - _roi.height / 2.0f;
}
// -----------------------------------------------------------------------
// Se verifica si es posible realizar escalado de la ROI
bool KCFTracker::verificarEscalado(float ajuste) const
{   // Tamano que tendria la ROI con el reajuste:
    const float size = std::sqrt(_roi.width * _roi.height) * ajuste;
    // Tamano relativo al inicial:
    const float tamano_relativo = size / _init_size;
    // Retorna un booleano para verificar si esta o no dentro del rango.
    return tamano_relativo >= min_scale && tamano_relativo <= max_scale;
}
// -----------------------------------------------------------------------
// DFT por canal. Cada fila de `x` es un canal de caracteristicas sobre la rejilla de
// celdas, de modo que basta reinterpretarla como una imagen 2D.
// reshape(1, alto) no copia nada: solo vuelve a "leer" la fila como una matriz de
// _cells.height filas.
void KCFTracker::spectrum(const cv::Mat &x, std::vector<cv::Mat> &xf) const
{
    xf.resize(_channels);
    for (int c = 0; c < _channels; c++)
        cv::dft(x.row(c).reshape(1, _cells.height), xf[c], cv::DFT_COMPLEX_OUTPUT);
}
// -----------------------------------------------------------------------
// Sigue al blanco en un cuadro nuevo. Los pasos son:
//  1) Buscar el blanco a la escala actual y, si esta activada la estimacion de escala,
//     tambien un poco mas chico y un poco mas grande. Quedarse con la mejor.
//  2) Calcular el PSR y decidir si el blanco esta perdido. Si lo esta, no se toca nada.
//  3) Si no esta perdido: actualizar la referencia del PSR, la escala y la posicion de
//     la caja, y entrenar el modelo con la apariencia nueva.
cv::Rect KCFTracker::update(const cv::Mat &image)
{
    clampROI(image);

    // Centro actual de la caja (antes de moverla).
    const float cx = _roi.x + _roi.width / 2.0f;
    const float cy = _roi.y + _roi.height / 2.0f;

    Response best = detect(getFeatures(image, false, 1.0f));
    float best_adjust = 1.0f;
    float best_score = best.peak; // la escala actual no lleva penalizacion

    if (scale_step != 1.0f)
    {
        // Se prueba la caja un paso mas chica (1 / scale_step) y un paso mas grande.
        const float candidates[2] = {1.0f / scale_step, scale_step};
        for (int i = 0; i < 2; i++)
        {
            const float adjust = candidates[i];
            if (!verificarEscalado(adjust))
                continue;

            const Response r = detect(getFeatures(image, false, adjust));

            // Ambos candidatos de escala llevan la misma penalizacion, de modo que
            // compiten entre si en igualdad de condiciones y solo le ganan a la escala
            // actual si mejoran el pico en mas de 1/scale_weight. Un update nunca puede
            // saltar dos pasos de escala.
            const float score = scale_weight * r.peak;
            if (score > best_score)
            {
                best = r;
                best_score = score;
                best_adjust = adjust;
            }
        }
    }

    _psr = best.psr;

    // Piso absoluto, mas un criterio relativo a la propia historia del seguidor. El
    // relativo es el que detecta oclusiones: la escala absoluta del PSR depende de la
    // escena, del tamano del blanco y del contraste, asi que un umbral fijo o no salta
    // nunca o salta siempre.
    const bool below_floor = _psr < psr_threshold;
    const bool collapsed = psr_drop_ratio > 0.0f && _psr_ref_n >= kPsrWarmup &&
                           _psr < psr_drop_ratio * _psr_ref;
    _lost = below_floor || collapsed;

    // Todavia no se ha confirmado nada, de modo que un blanco perdido deja la caja, la
    // escala y el modelo exactamente donde estaban. Eso mantiene la ventana de busqueda
    // anclada en la ultima posicion confiable en vez de dejarla derivar tras un pico
    // espurio, que es lo que permite volver a engancharse cuando el blanco reaparece.
    if (_lost)
        return cv::Rect(_roi);

    // Promedio exponencial del PSR: cada cuadro nuevo aporta un 5 % (kPsrRefRate) y el
    // 95 % restante viene de la historia. El primer cuadro se toma tal cual.
    _psr_ref = (_psr_ref_n == 0) ? _psr : (1.0f - kPsrRefRate) * _psr_ref + kPsrRefRate * _psr;
    if (_psr_ref_n < kPsrWarmup)
        _psr_ref_n++;

    _scale *= best_adjust;
    _roi.width *= best_adjust;
    _roi.height *= best_adjust;

    // Recentrar sobre la deteccion. El desplazamiento del pico viene en celdas HOG del
    // parche extraido, de modo que una celda son cell_size * _scale pixeles de imagen.
    _roi.x = cx - _roi.width / 2.0f + best.shift.x * cell_size * _scale;
    _roi.y = cy - _roi.height / 2.0f + best.shift.y * cell_size * _scale;

    clampROI(image);
    // Se aprende la apariencia en la nueva posicion, mezclandola suavemente con la
    // anterior (interp_factor es la fraccion de lo nuevo).
    train(getFeatures(image, false), interp_factor);

    return cv::Rect(_roi);
}
// -----------------------------------------------------------------------
// Kernel gaussiano para todos los desplazamientos ciclicos entre dos parches, ec. (12).
// Ambos entran por su espectro por canal y su norma cuadrada, que es lo que permite
// reutilizar el espectro del modelo entre los tres candidatos de escala.
//
// El kernel gaussiano mide que tan parecidos son dos parches: vale 1 si son identicos y
// baja hacia 0 a medida que se diferencian. Se basa en la distancia cuadrada entre ellos,
// que se puede escribir como ||x1||^2 + ||x2||^2 - 2 (x1 * x2). Lo caro seria calcular el
// producto x1 * x2 para cada desplazamiento posible, pero en frecuencia eso es solo una
// multiplicacion elemento a elemento (la correlacion cruzada), y se obtiene para todos
// los desplazamientos de una vez.
cv::Mat KCFTracker::kernelCorrelation(const std::vector<cv::Mat> &x1f, double x1_sqnorm,
                                      const std::vector<cv::Mat> &x2f, double x2_sqnorm) const
{
    cv::Mat c = cv::Mat::zeros(_cells.height, _cells.width, CV_32F);

    cv::Mat prod, corr;
    for (int i = 0; i < _channels; i++)
    {
        // X1 . conj(X2): correlacion cruzada del canal i, sumada sobre canales.
        // (El "true" final de mulSpectrums indica que se use el conjugado de X2).
        cv::mulSpectrums(x1f[i], x2f[i], prod, 0, true);
        cv::dft(prod, corr, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
        c += corr;
    }

    // Cantidad total de numeros en un parche: se usa para normalizar la distancia.
    const double n = (double)_cells.area() * _channels;

    // Distancia cuadrada normalizada. Por redondeo podria salir levemente negativa, asi
    // que se recorta en cero.
    cv::Mat d;
    cv::max((x1_sqnorm + x2_sqnorm - 2.0 * c) / n, 0.0, d);

    cv::Mat k;
    cv::exp(-d / (sigma * sigma), k);
    return k;
}
// -----------------------------------------------------------------------
// Busca el blanco en el parche z: f(z) = F^-1( k^xz . alpha^ ), ec. (18).
// Es decir: se compara z con el modelo para todos los desplazamientos (kxz), se pondera
// con los coeficientes aprendidos (alpha) y se vuelve al espacio de la imagen. El pico del
// resultado indica donde esta el blanco.
KCFTracker::Response KCFTracker::detect(const cv::Mat &z) const
{
    std::vector<cv::Mat> zf;
    spectrum(z, zf);

    // cv::sum(z.mul(z))[0] es ||z||^2 (la suma de los cuadrados de todos los elementos).
    const cv::Mat kxz = kernelCorrelation(zf, cv::sum(z.mul(z))[0], _xf, _x_sqnorm);

    cv::Mat kxzf, prod, f;
    cv::dft(kxz, kxzf, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums(_alphaf, kxzf, prod, 0);
    cv::dft(prod, f, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    // Solo para leer el pico: centrar el mapa deja el desplazamiento nulo en el medio.
    const cv::Mat response = fftShift(f);

    // Posicion (pi) y valor (pv) del maximo del mapa.
    cv::Point pi;
    double pv;
    cv::minMaxLoc(response, NULL, &pv, NULL, &pi);

    Response out;
    out.peak = (float)pv;
    out.psr = computePSR(response, pi, out.peak);

    // Vecinos ciclicos: el refinamiento nunca se salta, ni siquiera en el borde.
    const int rows = response.rows, cols = response.cols;
    const int xm = (pi.x - 1 + cols) % cols, xp = (pi.x + 1) % cols;
    const int ym = (pi.y - 1 + rows) % rows, yp = (pi.y + 1) % rows;

    // Posicion del pico con precision de fracciones de celda (ver subPixelPeak), por
    // separado en x (vecinos izquierdo y derecho) y en y (vecinos de arriba y de abajo).
    out.shift = cv::Point2f(
        pi.x + subPixelPeak(response.at<float>(pi.y, xm), out.peak, response.at<float>(pi.y, xp)),
        pi.y + subPixelPeak(response.at<float>(ym, pi.x), out.peak, response.at<float>(yp, pi.x)));

    // Como el mapa esta centrado, el desplazamiento se mide desde el medio.
    out.shift.x -= cols / 2.0f;
    out.shift.y -= rows / 2.0f;
    return out;
}
// -----------------------------------------------------------------------
// Entrena (o actualiza) el modelo con el parche x. learning_rate dice cuanto pesa lo
// nuevo: 1 reemplaza el modelo por completo, y valores chicos lo actualizan de a poco.
void KCFTracker::train(const cv::Mat &x, float learning_rate)
{
    std::vector<cv::Mat> xf;
    spectrum(x, xf);
    const double sq = cv::sum(x.mul(x))[0];

    // k^xx: primera fila de la matriz kernel, ec. (17).
    const cv::Mat kxx = kernelCorrelation(xf, sq, xf, sq);

    cv::Mat kxxf;
    cv::dft(kxx, kxxf, cv::DFT_COMPLEX_OUTPUT);

    // alpha^ = y^ / (k^xx + lambda), ec. (17).
    //
    // Esta es la solucion de la regresion ridge en frecuencia: se buscan los
    // coeficientes que hacen que la respuesta sobre el parche de entrenamiento se
    // parezca lo mas posible al objetivo gaussiano y, y lambda evita que los
    // coeficientes crezcan sin control (si k^xx es casi cero en alguna frecuencia,
    // dividir por el daria numeros enormes).
    //
    // En la convencion del articulo, k^xx son los autovalores de una matriz kernel
    // circulante semidefinida positiva: reales y no negativos. El denominador queda
    // entonces acotado por debajo por lambda, que es exactamente para lo que esta el
    // regularizador. El puerto original recentraba y y k a la vez, lo que multiplica
    // k^xx por (-1)^(u+v) y convierte el denominador en (lambda - k^xx) en la mitad del
    // espectro: ahi el regularizador resta en vez de sumar, y se anula cuando
    // k^xx == lambda.
    const cv::Mat alphaf = complexDivision(_yf, kxxf + cv::Scalar(lambda, 0));

    if (learning_rate >= 1.0f)
    {
        // Entrenamiento desde cero: el modelo es exactamente este parche.
        _x = x.clone();
        _xf = xf;
        _alphaf = alphaf;
    }
    else
    {
        // Actualizacion suave: nuevo = (1 - tasa) * viejo + tasa * actual.
        const float a = 1.0f - learning_rate;
        const float b = learning_rate;

        _x = a * _x + b * x;
        // La DFT es lineal, de modo que mezclar los espectros equivale a transformar la
        // mezcla: el espectro en cache sigue siendo exactamente el de _x, sin pagar una
        // FFT extra por canal.
        for (int c = 0; c < _channels; c++)
            _xf[c] = a * _xf[c] + b * xf[c];
        _alphaf = a * _alphaf + b * alphaf;
    }

    _x_sqnorm = cv::sum(_x.mul(_x))[0];
}
// -----------------------------------------------------------------------
// Objetivo de regresion gaussiano y, ec. (1), ya en frecuencia.
//
// Es la "respuesta ideal" que deberia dar el filtro sobre el parche de entrenamiento:
// una campana con su maximo donde esta el blanco (desplazamiento cero) y que baja
// suavemente al alejarse. El ancho de la campana es proporcional al tamano del blanco.
cv::Mat KCFTracker::createGaussianTarget(int rows, int cols) const
{
    cv::Mat_<float> y(rows, cols);

    const float output_sigma = std::sqrt((float)rows * cols) / padding * output_sigma_factor;
    const float mult = -0.5f / (output_sigma * output_sigma);

    // Convencion del articulo: el pico va en el indice (0,0) y la distancia es la
    // ciclica. Asi y^ queda real y el mapa de respuesta sale en la misma base que k^xz,
    // sin recentrar nada.
    // "Distancia ciclica" significa que, por ejemplo, la fila rows - 1 esta a distancia 1
    // de la fila 0 (dando la vuelta), por eso se usa min(i, rows - i).
    for (int i = 0; i < rows; i++)
    {
        const int ih = std::min(i, rows - i);
        for (int j = 0; j < cols; j++)
        {
            const int jh = std::min(j, cols - j);
            y(i, j) = std::exp(mult * (float)(ih * ih + jh * jh));
        }
    }

    cv::Mat yf;
    cv::dft(y, yf, cv::DFT_COMPLEX_OUTPUT);
    return yf;
}
// -----------------------------------------------------------------------
// Extraer la ventana de busqueda alrededor del blanco y convertirla en caracteristicas
// HOG, ya multiplicadas por la ventana coseno.
//
// La ventana es "padding" veces mas grande que el blanco, para tener margen si el blanco
// se mueve. Siempre se lleva al mismo tamano fijo (la plantilla), sin importar que tan
// grande sea el blanco en la imagen: asi las cuentas cuestan siempre lo mismo.
// scale_adjust permite extraer la ventana un poco mas chica o mas grande, para probar
// otras escalas.
cv::Mat KCFTracker::getFeatures(const cv::Mat &image, bool initHann, float scale_adjust)
{
    if (initHann)
    {
        // Primera vez (desde init): se decide el tamano de la plantilla.
        const int padded_w = (int)(_roi.width * padding);
        const int padded_h = (int)(_roi.height * padding);

        // Ajustar el lado mayor a template_size; todo se sigue a esa escala.
        _scale = std::max(padded_w, padded_h) / (float)template_size;
        _tmpl_sz.width = (int)(padded_w / _scale);
        _tmpl_sz.height = (int)(padded_h / _scale);

        // Redondear a un numero entero y par de celdas HOG.
        _tmpl_sz.width = (_tmpl_sz.width / (2 * cell_size)) * 2 * cell_size + 2 * cell_size;
        _tmpl_sz.height = (_tmpl_sz.height / (2 * cell_size)) * 2 * cell_size + 2 * cell_size;

        // La normalizacion descarta un borde de una celda, de modo que menos de 4 celdas
        // por lado dejarian la rejilla vacia. Solo alcanzable con un roi muy alargado.
        _tmpl_sz.width = std::max(_tmpl_sz.width, 4 * cell_size);
        _tmpl_sz.height = std::max(_tmpl_sz.height, 4 * cell_size);
    }

    // Ventana en pixeles de imagen, centrada en el centro de la caja actual.
    cv::Rect window;
    window.width = (int)(scale_adjust * _scale * _tmpl_sz.width);
    window.height = (int)(scale_adjust * _scale * _tmpl_sz.height);
    window.x = (int)(_roi.x + _roi.width / 2.0f - window.width / 2.0f);
    window.y = (int)(_roi.y + _roi.height / 2.0f - window.height / 2.0f);

    cv::Mat patch = subwindow(image, window, cv::BORDER_REPLICATE);
    if (patch.size() != _tmpl_sz)
        cv::resize(patch, patch, _tmpl_sz);

    cv::Size cells;
    cv::Mat features = extractHOG(patch, cell_size, cells);
    _cells = cells;
    _channels = features.rows;

    if (initHann)
        createHanningMats();

    return _hann.mul(features);
}
// -----------------------------------------------------------------------
// Ventana coseno, replicada una vez por canal para calzar con el formato de las
// caracteristicas.
//
// Para que sirve? El KCF trata el parche como si se repitiera ciclicamente (como un
// papel tapiz), asi que el borde derecho "toca" al izquierdo. Eso crea saltos falsos en
// los bordes que confunden al filtro. La ventana coseno vale 1 en el centro y baja
// suavemente a 0 en los bordes, asi que al multiplicarla por las caracteristicas los
// bordes se apagan, los saltos desaparecen y el filtro se concentra en el centro, que es
// donde esta el blanco.
void KCFTracker::createHanningMats()
{
    // Ventana 1D horizontal (una fila) y vertical (una columna).
    cv::Mat hann1t(1, _cells.width, CV_32F);
    cv::Mat hann2t(_cells.height, 1, CV_32F);

    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float>(0, i) = 0.5f * (1 - std::cos(2 * (float)CV_PI * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float>(i, 0) = 0.5f * (1 - std::cos(2 * (float)CV_PI * i / (hann2t.rows - 1)));

    // Columna por fila da la ventana 2D (el producto de ambas ventanas 1D).
    const cv::Mat hann2d = hann2t * hann1t;
    // Se aplana en una sola fila y se repite una vez por canal, porque las
    // caracteristicas se guardan con una fila por canal.
    const cv::Mat row = hann2d.reshape(1, 1);

    cv::repeat(row, _channels, 1, _hann);
}

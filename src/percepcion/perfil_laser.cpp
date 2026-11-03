#include "perfil_laser.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>

PerfilLaser::PerfilLaser(const Intrinsecos &cam, const ConfigLaser &cfg)
    : cam_(cam), cfg_(cfg), tr_(cfg.spot), siguiendo_antes_(false), desde_barrida_(0)
{
    retadores_iniciar(&ret_, &cfg_.barrida);
}
//---------------------------------------------------------------------------
// Busca en toda la imagen sin la pista del Kalman (que ya puede haber aprendido un falso
// positivo) y le entrega los candidatos a los retadores. Si uno gana, el seguidor se siembra
// en el, en una ventana del tamano de un blob.
bool PerfilLaser::barrer(const cv::Mat &img)
{
    cont_.barridas++;
    const int n = tr_.candidatosGlobales(img, tr_.posicion(), cfg_.barrida.radio_exclusion, cand_);
    PuntoPx pts[16];
    const int np = std::min(n, 16);
    for (int i = 0; i < np; i++)
        pts[i] = PuntoPx{cand_[i].centro.x, cand_[i].centro.y};
    const PuntoPx blanco{tr_.posicion().x, tr_.posicion().y};
    PuntoPx nuevo;
    if (!retadores_barrida(&ret_, blanco, pts, np, &nuevo))
        return false;
    const int r = (int)std::ceil(cfg_.spot.radio_blob_max) + 2;
    if (!tr_.init(cv::Rect((int)nuevo.x - r, (int)nuevo.y - r, 2 * r + 1, 2 * r + 1), img))
        return false;
    cont_.cambios_barrida++;
    return true;
}
//---------------------------------------------------------------------------
// Procesa un cuadro y devuelve la medicion para el Kalman.
Medicion PerfilLaser::procesar(const Cuadro &c, const Pista &p)
{
    // Si el seguidor tiene una posicion que seguir (no esta buscando desde cero), se
    // corre su prediccion lo mismo que se corrio la escena por el giro de la torreta. Asi
    // la ventana de busqueda queda donde realmente deberia aparecer el blanco.
    if (tr_.estado() != SpotTracker::BUSCANDO)
    {
        const cv::Point2f e = desplazamientoEscena(cam_, tr_.posicion(), c.mov);
        tr_.shift(e.x, e.y);
        retadores_trasladar(&ret_, e.x, e.y);
    }
    // Si en este cuadro toca buscar en toda la imagen (o el seguidor se pierde dentro de
    // este update), que empiece por donde el Kalman espera al blanco.
    if (p.valida)
        tr_.sugerir(p.s, p.radio);

    tr_.update(c.img);

    // Barrida global cada periodo_cuadros mientras se sigue. Si cambia de blanco, la medicion
    // de este cuadro es la del retador y cuenta como enganche nuevo (el Kalman se reinicia
    // sobre ella si no la explica).
    if (tr_.estado() != SpotTracker::SIGUIENDO)
    {
        retadores_vaciar(&ret_);
        desde_barrida_ = 0;
    }
    else if (cfg_.barrida.habilitada && ++desde_barrida_ >= cfg_.barrida.periodo_cuadros)
    {
        desde_barrida_ = 0;
        if (barrer(c.img))
            siguiendo_antes_ = false;
    }

    Medicion m;
    cv::Point2f z;
    const bool siguiendo = tr_.estado() == SpotTracker::SIGUIENDO;
    // medicion() solo devuelve true si se esta siguiendo y hubo deteccion en este cuadro.
    if (tr_.medicion(z))
    {
        const SpotTracker::Deteccion &d = tr_.ultima();
        // Factor de crecimiento de la sigma segun el SNR: vale 1 si el SNR es alto (igual
        // o mayor que snr_ref), crece como snr_ref / snr si es menor, y se topa en
        // sigma_max_factor. El max con 1e-3 evita dividir por cero.
        const float f = std::min(std::max(cfg_.snr_ref / std::max(d.snr, 1e-3f), 1.0f),
                                 cfg_.sigma_max_factor);
        // Con el blanco en movimiento el centroide es el de la mitad de la exposicion, no
        // el del instante del cuadro: hay hasta media estela de diferencia. El largo de la
        // estela sale de los ejes del blob (una recta de largo L tiene desviacion
        // L/sqrt(12)) y se suma como incertidumbre, sin suponer un tiempo de exposicion.
        //
        // Explicado con mas calma: el blob tiene forma de elipse, con una desviacion en su
        // eje mayor y otra en su eje menor. Un punto quieto seria redondo (ambos ejes
        // iguales). Si se alarga, es por la estela, y la parte del eje mayor que "sobra"
        // es eje_mayor^2 - eje_menor^2 (en varianza). Como una recta de largo L tiene
        // desviacion L / raiz(12), el largo de la estela es raiz(12) * raiz(sobra), y
        // raiz(12) = 3.464.
        const float estela = 3.464f * std::sqrt(std::max(d.eje_mayor * d.eje_mayor - d.eje_menor * d.eje_menor, 0.0f));
        const float s_base = cfg_.sigma_px * f, s_estela = 0.5f * estela;
        m.valida = true;
        m.z = z;
        // Las dos incertidumbres (la del detector y la de la estela) son independientes,
        // asi que se combinan sumando sus varianzas.
        m.sigma = std::sqrt(s_base * s_base + s_estela * s_estela);
        m.confianza = std::min(d.snr / cfg_.snr_ref, 1.0f);
        // Si en el cuadro anterior no se estaba siguiendo, este es un enganche nuevo.
        m.nuevo_enganche = !siguiendo_antes_;
        m.caja = cv::Rect2f(d.caja);
    }
    // Estadisticas: se cuenta un enganche al pasar de "no siguiendo" a "siguiendo", y una
    // perdida al reves.
    if (siguiendo && !siguiendo_antes_)
        cont_.enganches++;
    if (!siguiendo && siguiendo_antes_)
        cont_.perdidas++;
    siguiendo_antes_ = siguiendo;
    return m;
}
//---------------------------------------------------------------------------
// Suelta el blanco: el seguidor vuelve a buscar desde cero. La proxima llamada a
// procesar() traera la pista del Kalman, asi que la busqueda empezara por ahi.
void PerfilLaser::reenganchar(Motivo m)
{
    if (m == Motivo::RECHAZO_KALMAN)
        cont_.reenganches_kalman++;
    tr_.reset();
    siguiendo_antes_ = false;
    retadores_vaciar(&ret_);
    desde_barrida_ = 0;
}
//---------------------------------------------------------------------------
// Texto del estado actual, para mostrar en pantalla.
const char *PerfilLaser::estado() const
{
    switch (tr_.estado())
    {
    case SpotTracker::SIGUIENDO: return "SIGUIENDO";
    case SpotTracker::TENTATIVO: return "CONFIRMANDO";
    default: return "BUSCANDO";
    }
}
//---------------------------------------------------------------------------
// Dibuja sobre la imagen la ventana de busqueda (en amarillo) y, si hubo deteccion, su
// caja y una cruz en el centro: en verde si esta enganchado y en naranjo si todavia esta
// confirmando. Recordar que OpenCV usa colores en orden B, G, R.
void PerfilLaser::dibujar(cv::Mat &img) const
{
    if (tr_.ventana().area() > 0)
        cv::rectangle(img, tr_.ventana(), cv::Scalar(0, 255, 255), 1);
    if (tr_.detectado())
    {
        const SpotTracker::Deteccion &d = tr_.ultima();
        const cv::Scalar color = enganchado() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255);
        cv::rectangle(img, d.caja, color, 1);
        cv::drawMarker(img, d.centro, color, cv::MARKER_CROSS, 16, 1);
    }
    dibujarRetadores(img, ret_);
}

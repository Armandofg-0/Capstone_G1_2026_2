#include "adquisidor_rojo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

// Separa la imagen en sus canales B, G y R y calcula R - max(G, B). cv::subtract satura en 0:
// un resultado negativo queda en 0 en vez de dar la vuelta.
void mapaRojo(const cv::Mat &bgr, cv::Mat &out)
{
    cv::Mat c[3];
    cv::split(bgr, c);
    cv::subtract(c[2], cv::max(c[0], c[1]), out); // saturado en 0
}

// Pasos: umbral (pixeles bastante rojos), apertura (borra puntitos sueltos), cierre (une las
// piezas de un mismo avion) y componentes conexas (cada mancha separada recibe una etiqueta y
// sus estadisticas: area y caja). Cada mancha que toca la zona se devuelve con su confianza.
void detectarRojo(const cv::Mat &mapa, const cv::Rect &zona, const ConfigRojo &cfg, const cv::Mat &movimiento,
                  std::vector<Deteccion> &out)
{
    out.clear();
    if (cfg.peso_movimiento > 0.0f && movimiento.empty())
        return; // sin cuadros suficientes para la varianza, el rojo quieto ganaria la adquisicion
    // Se segmenta la imagen completa y se devuelven enteras las manchas que tocan la zona: si se
    // segmentara solo la zona, una zona chica (la verificacion de un KCF que se encogio) cortaria
    // al avion y lo volveria a sembrar chico.
    const cv::Rect z = zona & cv::Rect(0, 0, mapa.cols, mapa.rows);
    if (z.area() == 0)
        return;
    const cv::Mat &roi = mapa;
    cv::Mat mascara, etiquetas, stats, centros;
    cv::threshold(roi, mascara, cfg.umbral - 1, 255, cv::THRESH_BINARY);
    cv::morphologyEx(mascara, mascara, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    if (cfg.cierre > 1)
        cv::morphologyEx(mascara, mascara, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cfg.cierre, cfg.cierre)));
    const int n = cv::connectedComponentsWithStats(mascara, etiquetas, stats, centros, 8, CV_32S);
    const int area_max = (int)(cfg.area_max_frac * mapa.total());
    cv::Mat movil;
    const bool con_mov = !movimiento.empty() && cfg.peso_movimiento > 0.0f;
    if (con_mov)
        cv::dilate(movimiento, movil, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                   cv::Size(cfg.dilatacion_movimiento, cfg.dilatacion_movimiento)));
    for (int i = 1; i < n; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const cv::Rect r(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                         stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
        if (area < cfg.area_min || area > area_max || (r & z).area() == 0)
            continue;
        const double medio = cv::mean(roi(r), (etiquetas(r) == i) & (roi(r) >= cfg.umbral))[0];
        Deteccion d;
        d.caja = cv::Rect2f(r);
        d.clase = 0;
        d.prob = (float)std::min(1.0, medio / (2.0 * cfg.umbral));
        if (con_mov) {
            const cv::Mat propia = etiquetas(r) == i;
            const float frac = (float)cv::countNonZero(movil(r) & propia) / (float)area;
            d.prob *= 1.0f - cfg.peso_movimiento + cfg.peso_movimiento * frac;
        }
        out.push_back(d);
    }
}

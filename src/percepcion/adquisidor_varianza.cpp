// Detecta lo que se mueve en la escena aunque la camara tambien gire: lleva los cuadros
// anteriores al punto de vista del actual (compensando el giro de la torreta) y compara. Lo que
// cambia de verdad es un objeto en movimiento. Hoy se usa como desempate del detector de rojo.
// Adquisicion por varianza temporal (ver adquisidor_varianza.hpp).
#include "adquisidor_varianza.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

// Constructor: guarda la calibracion de la camara y los parametros.
AdquisidorVarianza::AdquisidorVarianza(const Intrinsecos &cam, const Config &cfg) : cam_(cam), cfg_(cfg) {}
//---------------------------------------------------------------------------
// Guarda el cuadro k en gris junto con la pose de la torreta al tomarlo. Se conservan solo los
// ultimos cuadros (una cola de largo fijo). El lock_guard toma el mutex (candado entre hilos)
// y lo suelta solo al salir de la funcion: registrar lo llama el lazo y detectar otro hilo.
void AdquisidorVarianza::registrar(int64_t k, const cv::Mat &img, const double q[2])
{
    Registro r;
    r.k = k;
    if (img.channels() == 3)
        cv::cvtColor(img, r.gris, cv::COLOR_BGR2GRAY);
    else
        r.gris = img.clone();
    r.q[0] = q[0];
    r.q[1] = q[1];
    std::lock_guard<std::mutex> lk(m_);
    regs_.push_back(std::move(r));
    // Se guardan los cuadros que pueden pedirse: los ultimos 2s + latencia del adquisidor.
    while (regs_.size() > (size_t)(4 * cfg_.separacion + 16))
        regs_.pop_front();
}
//---------------------------------------------------------------------------
// H = K R_b^T R_a K^-1 lleva pixeles del cuadro a al cuadro b (rotacion pura).
void AdquisidorVarianza::alinear(const Registro &a, const Registro &b, cv::Mat &out, cv::Mat &valido) const
{
    double Ra[9], Rb[9];
    R_0c(a.q[0], a.q[1], Ra);
    R_0c(b.q[0], b.q[1], Rb);
    const cv::Matx33d K(cam_.fu, 0, cam_.cu, 0, cam_.fv, cam_.cv, 0, 0, 1);
    const cv::Matx33d MRa(Ra), MRb(Rb);
    const cv::Matx33d H = K * MRb.t() * MRa * K.inv();
    cv::warpPerspective(a.gris, out, H, a.gris.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::warpPerspective(cv::Mat(a.gris.size(), CV_8U, cv::Scalar(255)), valido, H, a.gris.size(),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
}
//---------------------------------------------------------------------------
// Mascara de movimiento del cuadro k: diferencia de tres cuadros (k, k-s y k-2s) ya alineados.
// Un pixel cuenta como "movil" solo si cambio respecto de los dos anteriores; asi no se marca
// el lugar donde el objeto estaba antes, solo donde esta ahora.
bool AdquisidorVarianza::movimiento(int64_t k, cv::Mat &mascara)
{
    Registro r0, r1, r2;
    int encontrados = 0;
    {
        std::lock_guard<std::mutex> lk(m_);
        for (const Registro &r : regs_)
        {
            if (r.k == k)
                r0 = r, encontrados++;
            else if (r.k == k - cfg_.separacion)
                r1 = r, encontrados++;
            else if (r.k == k - 2 * cfg_.separacion)
                r2 = r, encontrados++;
        }
    }
    if (encontrados < 3)
        return false;
    cv::Mat w1, v1, w2, v2, d1, d2;
    alinear(r1, r0, w1, v1);
    alinear(r2, r0, w2, v2);
    cv::absdiff(r0.gris, w1, d1);
    cv::absdiff(r0.gris, w2, d2);
    mascara = (d1 > cfg_.umbral) & (d2 > cfg_.umbral) & v1 & v2;
    return true;
}
//---------------------------------------------------------------------------
// Manchas de movimiento dentro de la zona, como detecciones (el uso original, hoy sin uso en
// el lazo: el detector principal es el de rojo).
void AdquisidorVarianza::detectar(const cv::Rect &zona, int64_t k, std::vector<Deteccion> &out)
{
    out.clear();
    cv::Mat mascara;
    if (!movimiento(k, mascara))
        return;
    const cv::Rect z = zona & cv::Rect(0, 0, mascara.cols, mascara.rows);
    if (z.area() == 0)
        return;
    cv::Mat m = mascara(z);
    // Apertura para quitar ruido aislado y dilatacion para juntar las partes del blanco.
    cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::dilate(m, m, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    cv::Mat etiquetas, stats, centros;
    const int n = cv::connectedComponentsWithStats(m, etiquetas, stats, centros, 8, CV_32S);
    const float area_max = cfg_.area_max_frac * z.area();
    for (int i = 1; i < n; ++i)
    {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < cfg_.area_min || area > area_max)
            continue;
        Deteccion d;
        d.caja = cv::Rect2f((float)(z.x + stats.at<int>(i, cv::CC_STAT_LEFT)),
                            (float)(z.y + stats.at<int>(i, cv::CC_STAT_TOP)),
                            (float)stats.at<int>(i, cv::CC_STAT_WIDTH), (float)stats.at<int>(i, cv::CC_STAT_HEIGHT));
        d.clase = 0;
        d.prob = std::min(1.0f, (float)area / (4.0f * cfg_.area_min));
        out.push_back(d);
    }
}

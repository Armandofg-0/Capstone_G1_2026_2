#include "percepcion.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <string>

// Cuanto se desplaza en la imagen un punto fijo de la escena por culpa del giro de la
// torreta. Es la misma cuenta que hace el Kalman con G: L_eff (en px/rad) multiplicado
// por la velocidad media de la torreta (rad/s) y por la duracion del intervalo (s).
cv::Point2f desplazamientoEscena(const Intrinsecos &cam, cv::Point2f pixel, const MovimientoPropio &m)
{
    // Sin movimiento conocido (Ts = 0), no hay desplazamiento.
    if (m.Ts <= 0.0)
        return cv::Point2f(0.0f, 0.0f);
    // L_eff se vuelve absurdo lejos del cuadro, y ahi tampoco importa: cada coordenada se acota
    // a un rectangulo del doble del tamano de la imagen, centrado en el punto principal.
    double s[2] = {std::min(std::max((double)pixel.x, cam.cu - cam.width), cam.cu + cam.width),
                   std::min(std::max((double)pixel.y, cam.cv - cam.height), cam.cv + cam.height)};
    double L[4];
    L_eff(&cam, s, m.theta_t, L);
    // Desplazamiento = Ts * L * qd (producto de la matriz 2x2 L por el vector qd).
    return cv::Point2f((float)(m.Ts * (L[0] * m.qd[0] + L[1] * m.qd[1])),
                       (float)(m.Ts * (L[2] * m.qd[0] + L[3] * m.qd[1])));
}
//---------------------------------------------------------------------------
// Magenta en orden B, G, R.
void dibujarRetadores(cv::Mat &img, const Retadores &rs)
{
    const cv::Scalar magenta(255, 0, 255);
    for (int i = 0; i < rs.n; i++)
    {
        const cv::Point2f p(rs.r[i].ultimo.x, rs.r[i].ultimo.y);
        cv::circle(img, p, 10, magenta, 1);
        cv::putText(img, std::to_string(rs.r[i].confirmaciones), p + cv::Point2f(12.0f, -12.0f),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, magenta, 1);
    }
}
//---------------------------------------------------------------------------
bool cajaEnBorde(const cv::Rect2f &caja, int ancho, int alto, float margen)
{
    return caja.x < margen || caja.y < margen || caja.x + caja.width > ancho - margen ||
           caja.y + caja.height > alto - margen;
}

// Calibracion de la camara (ver calibracion_camara.hpp).
#include "calibracion_camara.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

CalibracionCamara::CalibracionCamara(int cols, int filas) : patron_(cols, filas) {}
//--------------------------------------------------------------------------------------------------
// Esquinas del tablero con precision subpixel; todas las fotos deben tener el mismo tamano.
bool CalibracionCamara::capturar(const cv::Mat &img)
{
    if (img.empty() || (!esquinas_.empty() && img.size() != tamano_))
        return false;
    cv::Mat gris;
    if (img.channels() == 3)
        cv::cvtColor(img, gris, cv::COLOR_BGR2GRAY);
    else
        gris = img;
    std::vector<cv::Point2f> e;
    if (!cv::findChessboardCorners(gris, patron_, e, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
        return false;
    cv::cornerSubPix(gris, e, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
    tamano_ = img.size();
    esquinas_.push_back(e);
    return true;
}
//--------------------------------------------------------------------------------------------------
// calibrateCamera con el tablero en z = 0 y cuadros de lado 1: para los intrinsecos la escala no
// importa (solo cambia la traslacion de cada vista).
ResultadoCalibracion CalibracionCamara::calcular() const
{
    ResultadoCalibracion r;
    r.fotos = fotos();
    if (r.fotos < 3)
        return r;
    std::vector<cv::Point3f> plano;
    for (int i = 0; i < patron_.height; i++)
        for (int j = 0; j < patron_.width; j++)
            plano.emplace_back((float)j, (float)i, 0.0f);
    const std::vector<std::vector<cv::Point3f>> objetos(esquinas_.size(), plano);
    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    r.rms = cv::calibrateCamera(objetos, esquinas_, tamano_, K, dist, rvecs, tvecs);
    r.fu = K.at<double>(0, 0);
    r.fv = K.at<double>(1, 1);
    r.cu = K.at<double>(0, 2);
    r.cv = K.at<double>(1, 2);
    r.k1 = dist.at<double>(0);
    r.k2 = dist.at<double>(1);
    r.ok = cv::checkRange(K) && r.fu > 0.0 && r.fv > 0.0;
    return r;
}
//--------------------------------------------------------------------------------------------------
void CalibracionCamara::borrar()
{
    esquinas_.clear();
}

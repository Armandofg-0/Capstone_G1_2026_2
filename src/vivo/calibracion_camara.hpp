// Calibracion de los intrinsecos de la camara desde el dashboard,
// con fotos de un tablero de ajedrez, esquinas con precision subpixel
// y calibrateCamera de OpenCV. El lazo la usa solo en el nivel 0, porque calcular tarda segundos.
#pragma once
#include <opencv2/core.hpp>
#include <vector>

struct ResultadoCalibracion {
    bool ok = false;     // false con menos de 3 fotos o si calibrateCamera no converge
    int fotos = 0;
    double rms = 0.0;    // error de reproyeccion (px)
    double fu = 0.0, fv = 0.0, cu = 0.0, cv = 0.0;
    double k1 = 0.0, k2 = 0.0; // distorsion radial (solo informativa: el modelo usa pinhole)
};

class CalibracionCamara {
public:
    // cols x filas: esquinas interiores del tablero.
    CalibracionCamara(int cols, int filas);
    // Busca el tablero en la imagen (BGR o gris); si lo encuentra, guarda sus esquinas y devuelve true.
    bool capturar(const cv::Mat &img);
    ResultadoCalibracion calcular() const;
    void borrar();
    int fotos() const { return (int)esquinas_.size(); }

private:
    cv::Size patron_, tamano_;
    std::vector<std::vector<cv::Point2f>> esquinas_;
};

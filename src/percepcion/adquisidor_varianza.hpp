// Adquisicion clasica del perfil objeto por varianza temporal (CONTEXTO_PROYECTO.md sec. 3.4:
// vision clasica, sin YOLO).
//
// Un blanco que se mueve en el mundo cambia la imagen de una forma que el giro de la camara
// no explica. Para separarlo del fondo, cada cuadro anterior se lleva al marco del cuadro
// actual con la homografia de rotacion pura H = K R_k^T R_a K^-1, que es exacta para un fondo
// lejano (el offset l_x mueve el fondo cercano unos pocos pixeles, que absorbe el umbral).
// Despues se hace la diferencia de tres cuadros, |I_k - W(I_{k-s})| y |I_k - W(I_{k-2s})|:
// solo donde ambas superan el umbral esta el blanco en su posicion actual (la diferencia de
// dos cuadros tambien marcaria donde estaba antes). Cada mancha con un area razonable es una
// deteccion.
//
// Necesita la pose q de cada cuadro (en el sistema real, de la telemetria con su timestamp).
// Es seguro llamarlo desde dos hilos: registrar() desde el lazo y detectar() desde el hilo
// del adquisidor.
#pragma once

#include "model.h"
#include "perfil_objeto.hpp"

#include <opencv2/core.hpp>

#include <deque>
#include <mutex>
#include <vector>

class AdquisidorVarianza
{
public:
    struct Config
    {
        int separacion = 3;          // s: cuadros entre las imagenes que se comparan
        int umbral = 25;             // diferencia minima de intensidad (niveles de 0 a 255)
        int area_min = 120;          // area minima de una mancha (px)
        float area_max_frac = 0.25f; // area maxima, como fraccion de la zona (luz que cambia)
    };

    AdquisidorVarianza(const Intrinsecos &cam, const Config &cfg);

    // Guarda el cuadro k (en gris) con la pose de la camara al tomarlo.
    void registrar(int64_t k, const cv::Mat &img, const double q[2]);

    // Mascara de movimiento del cuadro k (255 donde las dos diferencias superan el umbral), en el
    // marco del cuadro k. Devuelve false si todavia no hay cuadros suficientes.
    bool movimiento(int64_t k, cv::Mat &mascara);
    // Detecciones del cuadro k dentro de zona. Si no hay cuadros suficientes, no detecta nada.
    void detectar(const cv::Rect &zona, int64_t k, std::vector<Deteccion> &out);

private:
    struct Registro
    {
        int64_t k;
        cv::Mat gris;
        double q[2];
    };
    // Cuadro a llevado al marco del cuadro b, con su mascara de pixeles validos.
    void alinear(const Registro &a, const Registro &b, cv::Mat &out, cv::Mat &valido) const;

    Intrinsecos cam_;
    Config cfg_;
    std::mutex m_;
    std::deque<Registro> regs_;
};

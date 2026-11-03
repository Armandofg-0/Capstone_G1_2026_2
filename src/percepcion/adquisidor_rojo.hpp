// Adquisicion del perfil objeto por color: el avion de juguete es rojo saturado sobre un fondo
// bien iluminado. La varianza temporal (adquisidor_varianza) entra solo como desempate: un objeto
// rojo quieto del fondo puede ser mas rojo que el avion, pero no se mueve.
//
// El mapa de rojo es R - max(G, B), saturado a [0, 255]: un pixel blanco, gris, verde o azul da
// ~0 y el rojo del avion da ~110 quieto y 45 a 110 con estela (fotos y videos de 2026-10-27, AWB
// automatico y --ev 1; la mano no pasa de ~35). El perfil objeto trabaja entero sobre este mapa: el KCF lo
// sigue con fHOG (ve la silueta roja y no la textura del fondo) y el adquisidor lo umbraliza.
#pragma once
#include "perfil_objeto.hpp"
#include <opencv2/core.hpp>
#include <vector>

struct ConfigRojo
{
    int umbral = 45;       // exceso de rojo minimo de un pixel del blanco (0 a 255)
    int area_min = 60;     // area minima de una mancha (px)
    int cierre = 15;       // lado (px) del cierre morfologico que une las piezas de un avion con estela
    float area_max_frac = 0.6f; // area maxima, como fraccion de la imagen
    // Peso del movimiento en la confianza: prob = color * (1 - peso + peso * fraccion movil). Con
    // 0.6 una mancha quieta conserva el 40 %: sobre prob_min (un avion detenido se sigue
    // verificando), pero pierde la adquisicion contra una que se mueve. 0 = solo color.
    float peso_movimiento = 0.6f;
    int dilatacion_movimiento = 15; // lado (px): la diferencia solo marca los bordes del avion
};

// Mapa de rojo (CV_8UC1) de una imagen BGR.
void mapaRojo(const cv::Mat &bgr, cv::Mat &out);

// Manchas del mapa que tocan zona (enteras, aunque se salgan de ella), en pixeles de la imagen. La
// confianza crece con el exceso de rojo medio de la mancha y vale 1 desde 2 * umbral.
// movimiento: mascara del adquisidor de varianza para el mismo cuadro, o vacia si no la hay
// (entonces solo cuenta el color).
void detectarRojo(const cv::Mat &mapa, const cv::Rect &zona, const ConfigRojo &cfg, const cv::Mat &movimiento,
                  std::vector<Deteccion> &out);

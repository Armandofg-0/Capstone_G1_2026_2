/*M///////////////////////////////////////////////////////////////////////////////////////
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2010-2013, University of Nizhny Novgorod, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// HOG de Felzenszwalb, adaptado del modulo latentsvm de OpenCV ("lsvmc_featurepyramid.cpp").
// Se porto desde la antigua API en C de OpenCV (IplImage, que ya esta obsoleta) a cv::Mat,
// para que compile con OpenCV 4. El bloque de licencia de arriba es el del codigo
// original de OpenCV y debe conservarse tal cual.
//
// Que es HOG? Son las siglas de "Histogram of Oriented Gradients" (histograma de
// gradientes orientados). En vez de describir un parche de imagen por sus colores, lo
// describe por sus bordes: la imagen se divide en celdas pequenas y, en cada una, se
// cuenta cuanto borde hay en cada direccion. Esto es mucho mas robusto que los colores
// ante cambios de iluminacion, y por eso el KCF lo usa como "apariencia" del blanco.

#pragma once

#include <opencv2/core.hpp>

// Extrae las caracteristicas HOG de Felzenszwalb, que tienen 31 canales por celda.
//
// La imagen de entrada ("image") debe ser de 8 bits, con 1 o 3 canales, y conviene que
// sus dos dimensiones sean multiplos de cellSize. El parametro cellSize es el tamano de
// cada celda HOG en pixeles (4 es lo habitual para el KCF). En "cells" se devuelve el
// tamano de la rejilla de caracteristicas, medido en celdas.
//
// Devuelve una matriz CV_32F con una fila por canal de caracteristica y una columna por
// celda (las celdas ordenadas por filas), es decir, de 31 x (cells.width * cells.height).
// Se organiza asi para que quien la use pueda calcular una FFT 2D por canal simplemente
// reinterpretando una fila como una imagen.
cv::Mat extractHOG(const cv::Mat &image, int cellSize, cv::Size &cells);

// Los modos de funcionamiento del programa torreta (se eligen con general.modo), y las
// funciones que comparten entre ellos.

#pragma once

#include "config.hpp"

#include <chrono>
#include <vector>

// Modo "vivo": en tiempo real sobre la camara de la Pi, publicando al
// dashboard si esta habilitado. El bucle esta en src/vivo/correr_vivo.hpp.
int correrVivo(const Config &cfg);

// --- comun a los modos ----------------------------------------------------------------

// Percentil con interpolacion lineal (como np.percentile).
double percentil(std::vector<double> v, double p);

// Milisegundos transcurridos desde t0.
double msDesde(std::chrono::steady_clock::time_point t0);

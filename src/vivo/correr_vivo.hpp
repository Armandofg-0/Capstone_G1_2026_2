// BUCLE DEL MODO VIVO (general.modo = vivo): corre en tiempo real sobre una fuente de cuadros y
// publica cada cuadro al dashboard. En el nivel 0 solo publica la imagen con sus datos mínimos; la
// percepción, el EKF y el NMPC entran sobre este mismo bucle en los niveles siguientes (cadena.hpp).
#pragma once
#include "config.hpp"
#include "fuente.hpp"
#include <memory>
//--------------------------------------------------------------------------------------------------

// Corre el modo vivo con la fuente dada (sin abrir; nullptr cuenta como fuente no disponible).
// Termina tras vivo.segundos (si es > 0), con Ctrl+C o cuando la fuente falla. Al salir imprime los
// cuadros, los fps y el p50/p99 de us_publicar. Devuelve 0 si terminó por tiempo o por Ctrl+C, y 1
// si la fuente no abrió o falló, o si el dashboard no pudo escuchar.
int correrVivo(const Config &cfg, std::unique_ptr<Fuente> fuente);

// Combinaciones de [vivo] que tienen sentido: niveles 0 a 3 sobre una fuente real (el nivel 4 espera
// el enlace con la ESP32). Si no, lo explica y devuelve false.
bool vivoConfigValida(const ParamsVivo &v);

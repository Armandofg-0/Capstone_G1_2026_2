// Configuracion del programa torreta. Viene de dos fuentes: un archivo INI (por defecto
// config/torreta.ini) y asignaciones del tipo "seccion.clave=valor" escritas en la linea
// de comandos, que pisan lo que diga el archivo.
//
// El lector es el de src/params/ini.h. Lo que corre en C (camara, lazo, filtro, MPC y
// simulacion del control) vive en Params, con sus valores por defecto en params.c; aca solo
// quedan las opciones de la aplicacion y de la percepcion (C++). Una clave desconocida es
// un error: asi, un error de tipeo no pasa inadvertido.

#pragma once

#include "params.h"
#include "adquisidor_rojo.hpp"
#include "perfil_laser.hpp"
#include "perfil_general.hpp"
#include "perfil_objeto.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// Toda la configuracion junta. Cada bloque corresponde a una seccion del archivo INI.
struct Config
{
    // Para el laser, el canal por defecto es CROMA_ROJO (ver SpotTracker::Canal): destaca
    // el rojo del laser por sobre un fondo blanco o gris.
    Config() : p(params_por_defecto()) { laser.spot.canal = SpotTracker::CROMA_ROJO; }

    // [general]
    std::string modo = "vivo";       // unico modo
    std::string perfil = "laser";    // laser | objeto (avion de juguete) | general (recuadro del usuario)
    std::string csv;                 // registro por cuadro (vacio: no se escribe)
    // Fuente elegida al arrancar (picam o usb, ver elegirCamara). La fija quien llama a
    // cargarConfig: con usb, se aplica [usb].
    std::string fuente = "picam";
    // Valor cargado de cada clave de PARAMS_CLAVES_AJUSTE ("" si no esta en el INI), para el
    // dashboard.
    std::vector<std::pair<std::string, std::string>> ajustables;

    // [camara] [lazo] [kalman] [mpc] y la parte C de [simulacion]
    Params p;

    // [laser]
    ConfigLaser laser;

    // [objeto]
    ConfigObjeto objeto;
    ConfigRojo rojo; // adquisidor por color del perfil objeto ([objeto])
    std::string objeto_camara = "--ev 1"; // opciones de rpicam-vid del perfil objeto
    // [seleccion]: perfil general (KCF sobre el recuadro que marca el usuario)
    ConfigGeneral general;
    // [camara]: esquinas interiores del tablero para calibrar la camara desde el dashboard
    int tablero_cols = 6, tablero_filas = 9;
    std::string general_camara = "--ev 1";
};

// Lee el archivo (si `ruta` no esta vacia) y aplica las asignaciones en orden. Con
// cfg.fuente == "usb", las asignaciones de [usb] pisan a las del resto del archivo. Devuelve
// false e informa por stderr si el archivo no se puede leer, o si hay una clave
// desconocida o un valor invalido.
bool cargarConfig(const std::string &ruta, const std::vector<std::string> &asignaciones, Config &cfg);

// Ruta de un recurso del proyecto (config/): tal cual si existe relativa al
// directorio actual, y si no, relativa a la raiz del proyecto.
std::string rutaProyecto(const std::string &ruta);

// Perfil laser: para blancos puntuales de alto contraste (por ejemplo, un puntero laser
// sobre un fondo claro).
//
// La deteccion y el seguimiento los hace un mismo SpotTracker (basado en CFAR): primero
// busca en toda la imagen submuestreada, luego confirma al candidato en dos cuadros
// seguidos y, una vez confirmado, lo sigue buscando solo en una ventana a su alrededor.
// Todo corre en el hilo del lazo, porque la busqueda global cuesta alrededor de 1 ms en un
// PC. Mandarla a otro hilo no ahorraria nada y solo agregaria un cuadro de retardo.
//
// Lo que agrega este perfil por encima del SpotTracker:
//  - Traslada la prediccion del seguidor segun el movimiento propio de la torreta.
//  - Si hay que buscar de nuevo, busca primero alrededor de la pista del Kalman y prueba
//    los candidatos por cercania a ella, asi un destello lejano no le gana al blanco.
//  - La sigma de la medicion (su incertidumbre) crece cuando el SNR cae, para que el
//    Kalman le crea menos a las detecciones debiles.
//  - Atiende los pedidos de reenganche del Kalman: suelta el blanco y lo busca de nuevo
//    con la ayuda de la pista.

#pragma once

#include "percepcion.hpp"
#include "retador.h"
#include "spottracker.hpp"
#include <string>

struct ConfigLaser
{
    SpotTracker::Params spot;  // parametros del detector y seguidor CFAR

    // Sigma de la medicion cuando el blanco esta quieto y el SNR es alto. El centroide
    // que calcula el detector es muy preciso (a nivel de fracciones de pixel) sobre un
    // fondo liso, pero sobre un borde de alto contraste el halo del laser lo corre unos
    // pixeles. Por eso se usa 1.5 px, que es el mismo valor de sim.py con el que se
    // ajusto el Kalman.
    float sigma_px = 1.5f;

    float snr_ref = 12.0f;          // bajo este SNR, la sigma crece como snr_ref / snr
    float sigma_max_factor = 3.0f;  // pero nunca mas que este factor
    // Opciones de rpicam-vid: exposicion corta y ganancia baja para que el fondo quede oscuro
    // y el punto destaque en el canal rojo; AWB fijo para que el canal no cambie de escala.
    std::string camara = "--shutter 10000 --gain 1 --awbgains 1.6,1.5";
    ConfigBarrida barrida = barrida_por_defecto(); // busqueda global periodica (retador.h)
};

class PerfilLaser : public Perfil
{
public:
    PerfilLaser(const Intrinsecos &cam, const ConfigLaser &cfg);

    Medicion procesar(const Cuadro &c, const Pista &p) override;
    void reenganchar(Motivo m) override;
    const char *estado() const override;
    bool enganchado() const override { return tr_.estado() == SpotTracker::SIGUIENDO; }
    void dibujar(cv::Mat &img) const override;
    Contadores contadores() const override { return cont_; }

    const SpotTracker &seguidor() const { return tr_; }

private:
    // Barrida global: si un retador gana, siembra el seguidor en el y devuelve true.
    bool barrer(const cv::Mat &img);

    Intrinsecos cam_;
    ConfigLaser cfg_;
    SpotTracker tr_;
    bool siguiendo_antes_;   // si en el cuadro anterior se estaba siguiendo (para detectar cambios)
    Retadores ret_;          // retadores de la barrida global
    int desde_barrida_;      // cuadros siguiendo desde la ultima barrida
    std::vector<SpotTracker::Deteccion> cand_; // espacio de trabajo de la barrida
    Contadores cont_;
};

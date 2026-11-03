// Perfil general: sigue con un KCF lo que el usuario marca en el dashboard (un recuadro sobre
// el video). No tiene detector propio ni barrida: si el KCF se pierde, reintenta un tiempo
// cerca de la prediccion del Kalman sin aprender (para no contaminar la plantilla con el
// fondo) y, si no lo recupera, queda SIN_SELECCION hasta que se marque otro recuadro.
//
// La medicion es el centro de la caja, con sigma = sigma_px / psr_rel (acotado por
// sigma_max_factor), como el perfil objeto. Una caja a menos de margen_borde px del borde de la
// imagen no se mide: el blanco esta saliendo del cuadro y el centro de la caja no es el suyo.
#pragma once
#include "kcftracker.hpp"
#include "percepcion.hpp"
#include <memory>

struct ConfigGeneral
{
    KCFTracker kcf;                // prototipo con los parametros del KCF
    float sigma_px = 2.5f;         // sigma de la medicion con el PSR en su referencia
    float sigma_max_factor = 3.0f; // tope del crecimiento de sigma cuando el PSR cae
    int max_perdidos = 6;          // cuadros con el KCF perdido antes de pasar a reintentar
    int cuadros_reintento = 90;    // cuadros reintentando antes de soltar (3 s a 30 fps)
    int confirmaciones_reintento = 3; // cuadros seguidos con el KCF confiado para volver a medir
    float margen_borde = 4.0f;     // px; una caja mas cerca del borde no se mide
};

class PerfilGeneral : public Perfil
{
public:
    PerfilGeneral(const Intrinsecos &cam, const ConfigGeneral &cfg);

    // Recuadro marcado por el usuario (px del cuadro). Se siembra en el proximo procesar().
    void seleccionar(const cv::Rect &r);

    Medicion procesar(const Cuadro &c, const Pista &p) override;
    void reenganchar(Motivo m) override;
    const char *estado() const override;
    bool enganchado() const override { return estado_ == SIGUIENDO; }
    void dibujar(cv::Mat &img) const override;
    Contadores contadores() const override { return cont_; }

private:
    enum Estado { SIN_SELECCION, SIGUIENDO, REINTENTANDO };
    void soltar();

    Intrinsecos cam_;
    ConfigGeneral cfg_;
    std::unique_ptr<KCFTracker> kcf_;
    Estado estado_ = SIN_SELECCION;
    cv::Rect pedido_;          // recuadro pendiente de sembrar (area 0 = ninguno)
    int perdidos_ = 0, reintentos_ = 0, recuperados_ = 0;
    bool nuevo_ = false;       // la proxima medicion es la primera de un enganche
    Contadores cont_;
};

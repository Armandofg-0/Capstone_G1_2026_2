#include "perfil_general.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>

static cv::Point2f centro(const cv::Rect2f &r) { return cv::Point2f(r.x + 0.5f * r.width, r.y + 0.5f * r.height); }

PerfilGeneral::PerfilGeneral(const Intrinsecos &cam, const ConfigGeneral &cfg) : cam_(cam), cfg_(cfg) {}
//---------------------------------------------------------------------------
void PerfilGeneral::seleccionar(const cv::Rect &r)
{
    pedido_ = r;
}
//---------------------------------------------------------------------------
void PerfilGeneral::soltar()
{
    kcf_.reset();
    estado_ = SIN_SELECCION;
    perdidos_ = reintentos_ = 0;
}
//---------------------------------------------------------------------------
// 1. Un recuadro pendiente siembra un KCF nuevo. 2. El KCF avanza: en SIGUIENDO con el
// movimiento propio; en REINTENTANDO, centrado en la prediccion del Kalman si la hay (perdido,
// el KCF no aprende ni mueve su caja por si solo). 3. Se mide solo en SIGUIENDO, con el KCF
// confiado y la caja lejos del borde.
Medicion PerfilGeneral::procesar(const Cuadro &c, const Pista &p)
{
    Medicion m;
    const cv::Rect imagen(0, 0, c.img.cols, c.img.rows);
    if (pedido_.area() > 0)
    {
        const cv::Rect r = pedido_ & imagen;
        pedido_ = cv::Rect();
        if (r.width >= 8 && r.height >= 8)
        {
            kcf_.reset(new KCFTracker(cfg_.kcf));
            kcf_->init(r, c.img);
            estado_ = SIGUIENDO;
            perdidos_ = reintentos_ = 0;
            nuevo_ = true;
            cont_.enganches++;
            return m; // el KCF recien sembrado no mide en su propio cuadro
        }
    }
    if (!kcf_)
        return m;

    cv::Rect2f roi = kcf_->roi();
    if (estado_ == REINTENTANDO && p.valida)
    {
        roi.x = p.s.x - 0.5f * roi.width;
        roi.y = p.s.y - 0.5f * roi.height;
        kcf_->setROI(roi);
    }
    else
    {
        const cv::Point2f e = desplazamientoEscena(cam_, centro(roi), c.mov);
        kcf_->shift(e.x, e.y);
    }
    kcf_->update(c.img);

    if (kcf_->lost())
    {
        if (estado_ == SIGUIENDO && ++perdidos_ > cfg_.max_perdidos)
        {
            estado_ = REINTENTANDO;
            reintentos_ = 0;
            cont_.perdidas++;
        }
        else if (estado_ == REINTENTANDO && ++reintentos_ > cfg_.cuadros_reintento)
            soltar();
        recuperados_ = 0;
        return m;
    }
    perdidos_ = 0;
    if (estado_ == REINTENTANDO)
    {
        // Un cuadro confiado suelto puede ser el fondo parecido a la plantilla: hacen falta
        // varios seguidos para volver a medir.
        if (++recuperados_ < cfg_.confirmaciones_reintento)
        {
            ++reintentos_;
            return m;
        }
        estado_ = SIGUIENDO;
        nuevo_ = true;
        recuperados_ = 0;
    }
    const cv::Rect2f caja = kcf_->roi();
    if (cajaEnBorde(caja, c.img.cols, c.img.rows, cfg_.margen_borde))
        return m;

    const float ref = kcf_->psrReference();
    const float psr_rel = ref > 0.0f ? kcf_->psr() / ref : 1.0f;
    const float f = std::min(std::max(1.0f / std::max(psr_rel, 1e-3f), 1.0f), cfg_.sigma_max_factor);
    m.valida = true;
    m.z = centro(caja);
    m.sigma = cfg_.sigma_px * f;
    m.confianza = std::min(std::max(psr_rel, 0.0f), 1.0f);
    m.nuevo_enganche = nuevo_;
    m.caja = caja;
    nuevo_ = false;
    return m;
}
//---------------------------------------------------------------------------
// El Kalman rechazo varias mediciones seguidas: el KCF puede haberse ido a otra cosa. Sin
// detector propio, lo unico que se puede hacer es reintentar cerca de la prediccion.
void PerfilGeneral::reenganchar(Motivo m)
{
    if (m == Motivo::RECHAZO_KALMAN)
        cont_.reenganches_kalman++;
    if (!kcf_)
        return;
    if (m == Motivo::MANUAL)
        soltar();
    else if (estado_ == SIGUIENDO)
    {
        estado_ = REINTENTANDO;
        reintentos_ = 0;
    }
}
//---------------------------------------------------------------------------
const char *PerfilGeneral::estado() const
{
    switch (estado_)
    {
    case SIGUIENDO: return "SIGUIENDO";
    case REINTENTANDO: return "REINTENTANDO";
    default: return "SIN_SELECCION";
    }
}
//---------------------------------------------------------------------------
void PerfilGeneral::dibujar(cv::Mat &img) const
{
    if (!kcf_)
        return;
    const cv::Scalar color = estado_ == SIGUIENDO ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
    cv::rectangle(img, cv::Rect(kcf_->roi()), color, 2);
}

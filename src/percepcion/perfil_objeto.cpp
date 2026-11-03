#include "perfil_objeto.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

// Funciones auxiliares, visibles solo dentro de este archivo.
namespace {

// Milisegundos transcurridos desde el instante t0.
double msDesde(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
//---------------------------------------------------------------------------
// Centro de un rectangulo.
cv::Point2f centro(const cv::Rect2f &r)
{
    return cv::Point2f(r.x + 0.5f * r.width, r.y + 0.5f * r.height);
}
//---------------------------------------------------------------------------
// IoU ("interseccion sobre union") de dos cajas: el area que comparten dividida por el
// area que cubren entre las dos. Vale 1 si son identicas y 0 si no se tocan. El operador
// & entre rectangulos de OpenCV calcula su interseccion. El area de la union se obtiene
// sumando ambas areas y restando la interseccion (que si no, se contaria dos veces).
float iou(const cv::Rect2f &a, const cv::Rect2f &b)
{
    const float inter = (a & b).area();
    const float uni = a.area() + b.area() - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}
//---------------------------------------------------------------------------
// Cuadrado centrado en c, con "semilado" (la mitad del lado) dado, recortado a los bordes
// de la imagen.
cv::Rect cuadrado(cv::Point2f c, float semilado, const cv::Size &img)
{
    const int r = (int)std::ceil(semilado);
    return cv::Rect((int)std::floor(c.x) - r, (int)std::floor(c.y) - r, 2 * r + 1, 2 * r + 1) &
           cv::Rect(0, 0, img.width, img.height);
}
//---------------------------------------------------------------------------
// La deteccion de la red es el mismo objeto que sigue el KCF si su centro cae en la
// mitad central de la caja del KCF (un IoU bajo con el centro lejos es otro objeto que
// se superpone, no una caja corrida), o si el centro del KCF cae en la mitad central de la
// deteccion (el KCF se encogio sobre una parte del blanco).
bool mismoObjeto(const cv::Rect2f &kcf, const cv::Rect2f &det)
{
    const cv::Point2f d = centro(det) - centro(kcf);
    return (std::fabs(d.x) < 0.5f * kcf.width && std::fabs(d.y) < 0.5f * kcf.height) ||
           (std::fabs(d.x) < 0.5f * det.width && std::fabs(d.y) < 0.5f * det.height);
}

} // namespace

// Constructor: inicializa todo y lanza el hilo de la red, que queda esperando pedidos.
PerfilObjeto::PerfilObjeto(const Intrinsecos &cam, const ConfigObjeto &cfg, FuncionDeteccion detector)
    : cam_(cam), cfg_(cfg), detector_(detector), historial_((size_t)cfg.capacidad_historial),
      estado_(BUSCANDO), generacion_(0), visto_(-1), perdidos_(0), verif_fallidas_(0),
      intentos_locales_(0), ultima_verif_(-1), ultima_barrida_(-1), nuevo_(false), hay_caja_(false),
      hay_ultima_det_(false), salir_(false), hay_pedido_(false), en_curso_(false),
      hay_respuesta_(false)
{
    retadores_iniciar(&ret_, &cfg_.barrida);
    hilo_ = std::thread(&PerfilObjeto::bucle, this);
}
//---------------------------------------------------------------------------
// Destructor: le pide al hilo de la red que termine, lo despierta (por si esta dormido
// esperando un pedido) y espera a que efectivamente salga.
PerfilObjeto::~PerfilObjeto()
{
    {
        std::lock_guard<std::mutex> lk(m_);
        salir_ = true;
    }
    cv_.notify_all();
    if (hilo_.joinable())
        hilo_.join();
}

// =====================================================================================
// Hilo de la red

// Bucle del hilo de la red: duerme hasta que llega un pedido, lo atiende y deja la
// respuesta lista para que el lazo la recoja. Repite hasta que se pida salir.
void PerfilObjeto::bucle()
{
    for (;;)
    {
        Pedido p;
        // Se toma el pedido con el mutex tomado, pero se lo copia a una variable local
        // para poder soltar el mutex mientras la red trabaja (que tarda mucho).
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [this] { return salir_ || hay_pedido_; });
            if (salir_)
                return;
            p = pedido_;
            hay_pedido_ = false;
        }
        Respuesta r;
        r.pedido = p;
        atender(p, r); // el trabajo pesado, sin el mutex tomado
        {
            std::lock_guard<std::mutex> lk(m_);
            // std::move "traslada" la respuesta en lugar de copiarla. Hace falta porque
            // contiene un unique_ptr, que no se puede copiar (solo puede tener un dueno).
            respuesta_ = std::move(r);
            hay_respuesta_ = true;
        }
        cv_.notify_all();
    }
}
//---------------------------------------------------------------------------
// Atiende un pedido: corre la red, elige la deteccion que corresponde y, si hace falta,
// siembra un KCF nuevo sobre ella.
void PerfilObjeto::atender(const Pedido &p, Respuesta &r)
{
    Cuadro f;
    if (!historial_.obtener(p.indice, f))
        return; // el cuadro ya salio del historial: la respuesta llega vacia

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Deteccion> dets;
    detector_(f.img, p.zona, p.indice, dets);
    r.ms_red = msDesde(t0);
    // Se descartan las detecciones con confianza menor que prob_min. Es el patron
    // "erase-remove" de C++: remove_if junta al final los elementos a borrar, y erase los
    // elimina de una vez.
    dets.erase(std::remove_if(dets.begin(), dets.end(),
                              [this](const Deteccion &d) { return d.prob < cfg_.prob_min; }),
               dets.end());

    // Barrida: se devuelven todas, de la mas a la menos confiable; la decide el lazo.
    if (p.tipo == BARRIDA)
    {
        std::sort(dets.begin(), dets.end(), [](const Deteccion &a, const Deteccion &b) { return a.prob > b.prob; });
        r.todas = std::move(dets);
        return;
    }

    int elegida = -1; // indice de la deteccion elegida (-1 = ninguna)
    if (p.tipo == ADQUIRIR)
    {
        // Con pista del Kalman gana la deteccion mas cercana a la prediccion: en una zona
        // local cualquiera de las que haya, en la imagen completa solo si cae dentro del
        // radio. Sin pista (o sin nada cerca en la imagen completa), la mas confiable.
        if (p.con_prior)
        {
            float mejor = std::numeric_limits<float>::max();
            for (size_t i = 0; i < dets.size(); i++)
            {
                const cv::Point2f d = centro(dets[i].caja) - p.prior;
                const float dist = std::sqrt(d.dot(d));
                if ((p.local || dist <= p.radio_prior) && dist < mejor)
                {
                    mejor = dist;
                    elegida = (int)i;
                }
            }
        }
        // Si la busqueda era local y con pista, no se recurre a "la mas confiable": si no
        // se eligio nada arriba es porque no habia detecciones.
        if (elegida < 0 && !(p.local && p.con_prior))
            for (size_t i = 0; i < dets.size(); i++)
                if (elegida < 0 || dets[i].prob > dets[(size_t)elegida].prob)
                    elegida = (int)i;
    }
    else
    {
        // Verificacion: se elige la deteccion que mas se superpone con la caja del KCF.
        for (size_t i = 0; i < dets.size(); i++)
        {
            const float v = iou(p.caja, dets[i].caja);
            if (v > r.iou)
            {
                r.iou = v;
                elegida = (int)i;
            }
        }
    }
    if (elegida < 0)
        return;

    r.hay_det = true;
    r.det = dets[(size_t)elegida];
    // Adquisicion: siempre se siembra. Verificacion: solo si la caja del KCF se corrio
    // respecto de la de la red pero sigue siendo el mismo objeto.
    if (p.tipo == ADQUIRIR || (r.iou < cfg_.iou_ok && mismoObjeto(p.caja, r.det.caja)))
    {
        r.kcf = sembrar(r.det.caja, f, p, r.visto, r.ms_puesta);
    }
}
//---------------------------------------------------------------------------
// KCF nuevo sobre la caja de la red en el cuadro f, y puesto al dia con los cuadros que
// llegaron despues. En modo determinista llega exactamente hasta el cuadro anterior al
// de la entrega (espera a que el lazo los vaya agregando). En vivo, hasta el ultimo que
// habia al empezar: si persiguiera a los que siguen llegando podria no terminar nunca
// (con un video que se lee mas rapido que el KCF), y el resto lo hace el lazo.
std::unique_ptr<KCFTracker> PerfilObjeto::sembrar(const cv::Rect2f &caja, const Cuadro &f,
                                                  const Pedido &p, int64_t &visto, double &ms)
{
    auto t0 = std::chrono::steady_clock::now();
    // El KCF nuevo es una copia del prototipo (asi hereda sus parametros). Su caja inicial
    // se redondea a pixeles enteros, con al menos 1 px de ancho y alto.
    std::unique_ptr<KCFTracker> t(new KCFTracker(cfg_.kcf));
    const cv::Rect roi(cvRound(caja.x), cvRound(caja.y), std::max(1, cvRound(caja.width)),
                       std::max(1, cvRound(caja.height)));
    t->init(roi, f.img);
    visto = f.indice;
    ms = msDesde(t0);

    // Hasta que cuadro hay que ponerlo al dia (ver el comentario de arriba).
    const bool determinista = cfg_.latencia_cuadros > 0;
    const int64_t tope = determinista ? p.indice + cfg_.latencia_cuadros - 1 : historial_.ultimo();
    for (int64_t j = f.indice + 1; j <= tope; j++)
    {
        Cuadro cj;
        const bool hay = determinista ? historial_.esperar(j, cj, salir_) : historial_.obtener(j, cj);
        if (!hay)
            break;
        // Solo se mide el tiempo de computo del KCF, no el tiempo esperando cuadros.
        t0 = std::chrono::steady_clock::now();
        avanzar(*t, cj, nullptr);
        ms += msDesde(t0);
        visto = j;
    }
    return t;
}

// =====================================================================================
// Comunicacion entre hilos
//
// Solo puede haber un pedido a la vez. El lazo lo envia con enviar() y, mas adelante, lo
// recoge con recoger(). Mientras tanto, ocupado() devuelve true y no se envian otros.

// Envia un pedido a la red. Devuelve false (y no hace nada) si ya hay uno en curso.
bool PerfilObjeto::enviar(const Pedido &p)
{
    {
        std::lock_guard<std::mutex> lk(m_);
        if (en_curso_)
            return false;
        pedido_ = p;
        hay_pedido_ = true;
        en_curso_ = true;
    }
    cv_.notify_all(); // despierta al hilo de la red
    return true;
}
//---------------------------------------------------------------------------
// Recoge la respuesta del pedido en curso, si ya corresponde. Devuelve true si la
// entrego en r.
//  - En vivo: la respuesta se recoge apenas esta lista, sin esperar.
//  - En modo determinista: no se recoge antes de que pasen latencia_cuadros cuadros desde
//    el pedido; y al cumplirse ese plazo, se ESPERA a la red si todavia no termino. Asi el
//    resultado no depende de la velocidad real del computador.
bool PerfilObjeto::recoger(int64_t indice_actual, Respuesta &r)
{
    std::unique_lock<std::mutex> lk(m_);
    if (!en_curso_)
        return false;
    if (cfg_.latencia_cuadros > 0)
    {
        if (indice_actual < pedido_.indice + cfg_.latencia_cuadros)
            return false;
        cv_.wait(lk, [this] { return hay_respuesta_; });
    }
    else if (!hay_respuesta_)
        return false;
    r = std::move(respuesta_);
    hay_respuesta_ = false;
    en_curso_ = false;
    return true;
}
//---------------------------------------------------------------------------
// true si hay un pedido enviado cuya respuesta todavia no se recogio.
bool PerfilObjeto::ocupado() const
{
    std::lock_guard<std::mutex> lk(m_);
    return en_curso_;
}

// =====================================================================================
// Hilo del lazo

// Un paso del KCF en el cuadro c. La ventana va a la prediccion del Kalman si la hay y
// es coherente con la caja actual (recien enganchado, el Kalman todavia puede estar
// prediciendo el blanco anterior); si no, se traslada con el movimiento propio.
// "Coherente" significa que la prediccion cae a menos de media caja del centro actual.
void PerfilObjeto::avanzar(KCFTracker &t, const Cuadro &c, const Pista *p) const
{
    cv::Rect2f roi = t.roi();
    const cv::Point2f d = p ? p->s - centro(roi) : cv::Point2f();
    if (p && p->valida && std::sqrt(d.dot(d)) < 0.5f * std::max(roi.width, roi.height))
    {
        // Se mueve la caja para que quede centrada en la prediccion, sin cambiar su tamano.
        roi.x = p->s.x - 0.5f * roi.width;
        roi.y = p->s.y - 0.5f * roi.height;
        t.setROI(roi);
    }
    else
    {
        const cv::Point2f e = desplazamientoEscena(cam_, centro(roi), c.mov);
        t.shift(e.x, e.y);
    }
    t.update(c.img);
}
//---------------------------------------------------------------------------
// Procesa un cuadro y devuelve la medicion para el Kalman.
Medicion PerfilObjeto::procesar(const Cuadro &c, const Pista &p)
{
    // El cuadro se guarda en el historial antes que nada, porque la red podria necesitarlo.
    historial_.agregar(c);

    // Si llego la respuesta de un pedido anterior, se procesa.
    Respuesta r;
    if (recoger(c.indice, r))
        manejar(r);

    Medicion m;
    if (kcf_)
    {
        // Normalmente solo falta el cuadro actual; si el seguidor viene recien entregado
        // por la red, tambien los que ella no alcanzo a procesar.
        // Solo en el cuadro actual se usa la pista del Kalman (los anteriores ya pasaron,
        // y la pista es para este cuadro).
        bool completo = true;
        for (int64_t j = visto_ + 1; j <= c.indice && completo; j++)
        {
            if (j == c.indice)
                avanzar(*kcf_, c, cfg_.centrar_con_pista ? &p : nullptr);
            else
            {
                Cuadro cj;
                completo = historial_.obtener(j, cj);
                if (completo)
                    avanzar(*kcf_, cj, nullptr);
            }
            if (completo)
                visto_ = j;
        }
        // Los retadores se corren con la escena, como la ventana del KCF.
        const cv::Point2f e = desplazamientoEscena(cam_, centro(kcf_->roi()), c.mov);
        retadores_trasladar(&ret_, e.x, e.y);

        if (!completo)
        {
            soltar();   // se perdieron cuadros intermedios: el seguidor quedo atrasado
            estado_ = BUSCANDO;
        }
        else if (kcf_->lost())
        {
            // El KCF cree que perdio al blanco. Si pasa demasiados cuadros seguidos asi,
            // se pasa a reenganchar (solo desde SIGUIENDO, para contar una sola perdida).
            if (++perdidos_ > cfg_.max_perdidos && estado_ == SIGUIENDO)
            {
                estado_ = REENGANCHANDO;
                cont_.perdidas++;
                intentos_locales_ = 0;
            }
        }
        else
        {
            perdidos_ = 0;
            const cv::Rect2f caja = kcf_->roi();
            const bool en_borde = cajaEnBorde(caja, c.img.cols, c.img.rows, cfg_.margen_borde);
            // La sigma crece a medida que el PSR cae respecto de su propia historia; con el
            // perfil buscando de nuevo, al KCF se le cree poco.
            // psr_rel es el PSR actual relativo a su referencia: 1 es "lo normal" y valores
            // menores indican que el KCF esta menos seguro. El factor f vale 1/psr_rel,
            // acotado entre 1 y sigma_max_factor.
            const float ref = kcf_->psrReference();
            const float psr_rel = ref > 0.0f ? kcf_->psr() / ref : 1.0f;
            float f = std::min(std::max(1.0f / std::max(psr_rel, 1e-3f), 1.0f), cfg_.sigma_max_factor);
            if (estado_ == REENGANCHANDO)
                f = cfg_.sigma_max_factor;
            m.valida = !en_borde;
            m.z = centro(caja);
            m.sigma = cfg_.sigma_px * f;
            m.confianza = std::min(std::max(psr_rel, 0.0f), 1.0f);
            m.nuevo_enganche = nuevo_;
            m.caja = caja;
            nuevo_ = false;
            hay_caja_ = true;
            ultima_caja_ = caja;
        }
    }

    // Si la red esta libre, se le encarga trabajo: buscar el blanco si no se esta
    // siguiendo, o verificar al KCF si ya pasaron periodo_verificacion cuadros desde la
    // ultima verificacion.
    // Si no, y toca, una barrida global (la verificacion tiene prioridad: la barrida espera
    // al cuadro siguiente).
    if (estado_ != SIGUIENDO)
        retadores_vaciar(&ret_);
    if (!ocupado())
    {
        if (estado_ != SIGUIENDO)
            pedirAdquisicion(c, p);
        else if (cfg_.periodo_verificacion > 0 &&
                 c.indice - ultima_verif_ >= cfg_.periodo_verificacion)
            pedirVerificacion(c);
        else if (cfg_.barrida.habilitada && c.indice - ultima_barrida_ >= cfg_.barrida.periodo_cuadros)
            pedirBarrida(c);
    }
    return m;
}
//---------------------------------------------------------------------------
// Aplica la respuesta de la red.
void PerfilObjeto::manejar(Respuesta &r)
{
    // Estadisticas de tiempos.
    if (r.ms_red > 0.0)
        cont_.ms_red.push_back(r.ms_red);
    if (r.kcf)
        cont_.ms_puesta_al_dia.push_back(r.ms_puesta);
    if (r.hay_det)
    {
        ultima_det_ = r.det;
        hay_ultima_det_ = true;
    }

    if (r.pedido.tipo == ADQUIRIR)
    {
        // Si la red encontro el blanco y trajo un seguidor, este pasa a ser el actual y
        // se empieza a seguir desde cero (contadores reiniciados).
        if (r.kcf)
        {
            kcf_ = std::move(r.kcf);
            visto_ = r.visto;
            generacion_++;
            estado_ = SIGUIENDO;
            perdidos_ = 0;
            verif_fallidas_ = 0;
            intentos_locales_ = 0;
            ultima_verif_ = r.pedido.indice;   // la propia adquisicion vale como verificacion
            nuevo_ = true;
            hay_caja_ = true;
            ultima_caja_ = r.det.caja;
            cont_.enganches++;
        }
        else if (r.pedido.local)
            intentos_locales_++; // busqueda local fallida: se cuenta para pasar a global
        return;
    }
    if (r.pedido.tipo == BARRIDA)
    {
        manejarBarrida(r);
        return;
    }

    // Verificacion: solo cuenta si sigue siendo el mismo seguidor y se lo esta usando.
    if (r.pedido.generacion != generacion_ || !kcf_ || estado_ != SIGUIENDO)
        return;
    cont_.verificaciones++;
    if (r.hay_det && r.iou >= cfg_.iou_ok)
        verif_fallidas_ = 0; // la red coincide con el KCF: todo en orden
    else if (r.kcf)
    {
        // Mismo objeto, caja corrida: el KCF se re-siembra con la caja de la red (corrige
        // la deriva de posicion y de escala sin soltar el blanco).
        kcf_ = std::move(r.kcf);
        visto_ = r.visto;
        generacion_++;
        verif_fallidas_ = 0;
        cont_.re_siembras++;
    }
    else
    {
        // La red no ve al blanco donde esta el KCF (o ve otro objeto).
        cont_.verif_fallidas++;
        if (++verif_fallidas_ >= cfg_.max_verif_fallidas)
        {
            estado_ = REENGANCHANDO;
            cont_.perdidas++;
            intentos_locales_ = 0;
        }
    }
}
//---------------------------------------------------------------------------
// Arma y envia un pedido de adquisicion (buscar el blanco).
void PerfilObjeto::pedirAdquisicion(const Cuadro &c, const Pista &p)
{
    const cv::Size tam_img = c.img.size();
    Pedido q;
    q.tipo = ADQUIRIR;
    q.indice = c.indice;
    q.con_prior = p.valida;
    q.prior = p.s;
    q.radio_prior = std::max(p.radio, cfg_.radio_min_prior);
    q.zona = cv::Rect(0, 0, tam_img.width, tam_img.height); // por defecto, la imagen completa

    // Primero una zona acotada alrededor de donde deberia estar (pista del Kalman o, sin
    // ella, la ultima caja): entrada chica y rapida. Si no aparece, la imagen completa.
    if (intentos_locales_ < cfg_.max_intentos_locales && (p.valida || hay_caja_))
    {
        const cv::Point2f ctr = p.valida ? p.s : centro(ultima_caja_);
        // El tamano de la zona es el mayor entre el radio de la pista y margen_local veces
        // el tamano de la ultima caja conocida.
        const float lado = hay_caja_ ? std::max(ultima_caja_.width, ultima_caja_.height) : 0.0f;
        const float semilado = std::max(p.valida ? q.radio_prior : 0.0f, 0.5f * cfg_.margen_local * lado);
        const cv::Rect z = cuadrado(ctr, semilado, tam_img);
        // Solo vale la pena como busqueda local si es menos de media imagen; si no, se
        // busca directamente en la imagen completa.
        if (z.area() > 0 && z.area() < tam_img.area() / 2)
        {
            q.zona = z;
            q.local = true;
        }
    }
    if (enviar(q))
    {
        cont_.pedidos_red++;
        zona_pedida_ = q.zona;
    }
}
//---------------------------------------------------------------------------
// Arma y envia un pedido de verificacion: la red revisa un recorte cuadrado alrededor de
// la caja del KCF (de margen_verificacion veces su tamano, y como minimo 32 px de
// semilado).
void PerfilObjeto::pedirVerificacion(const Cuadro &c)
{
    Pedido q;
    q.tipo = VERIFICAR;
    q.indice = c.indice;
    q.caja = kcf_->roi();
    q.generacion = generacion_;
    q.local = true;
    const float lado = std::max(q.caja.width, q.caja.height) * cfg_.margen_verificacion;
    q.zona = cuadrado(centro(q.caja), std::max(0.5f * lado, 32.0f), c.img.size());
    if (enviar(q))
    {
        cont_.pedidos_red++;
        ultima_verif_ = c.indice;
        zona_pedida_ = q.zona;
    }
}
//---------------------------------------------------------------------------
// Arma y envia un pedido de barrida: la red busca en toda la imagen sin la pista del Kalman.
void PerfilObjeto::pedirBarrida(const Cuadro &c)
{
    Pedido q;
    q.tipo = BARRIDA;
    q.indice = c.indice;
    q.caja = kcf_->roi();
    q.generacion = generacion_;
    q.zona = cv::Rect(0, 0, c.img.cols, c.img.rows);
    if (enviar(q))
    {
        cont_.pedidos_red++;
        ultima_barrida_ = c.indice;
        zona_pedida_ = q.zona;
    }
}
//---------------------------------------------------------------------------
// Entrega los candidatos de la barrida a los retadores. Si uno gana, se pide una adquisicion
// local alrededor de el sobre el mismo cuadro de la barrida (sembrar() pone al dia el KCF
// nuevo) y, mientras llega, el KCF actual sigue con la sigma inflada (REENGANCHANDO).
void PerfilObjeto::manejarBarrida(const Respuesta &r)
{
    if (r.pedido.generacion != generacion_ || !kcf_ || estado_ != SIGUIENDO)
        return;
    cont_.barridas++;
    PuntoPx pts[16];
    const int n = std::min((int)r.todas.size(), 16);
    for (int i = 0; i < n; i++)
    {
        const cv::Point2f c = centro(r.todas[(size_t)i].caja);
        pts[i] = PuntoPx{c.x, c.y};
    }
    const cv::Point2f b = centro(r.pedido.caja);
    PuntoPx nuevo;
    if (!retadores_barrida(&ret_, PuntoPx{b.x, b.y}, pts, n, &nuevo))
        return;

    // Tamano de la zona: margen_local veces la caja del retador (la deteccion en nuevo).
    float lado = 0.0f;
    for (int i = 0; i < n; i++)
        if (pts[i].x == nuevo.x && pts[i].y == nuevo.y)
            lado = std::max(r.todas[(size_t)i].caja.width, r.todas[(size_t)i].caja.height);
    Pedido q;
    q.tipo = ADQUIRIR;
    q.indice = r.pedido.indice;
    q.local = true;
    q.con_prior = true;
    q.prior = cv::Point2f(nuevo.x, nuevo.y);
    q.radio_prior = cfg_.barrida.radio_asociacion;
    Cuadro f;
    const cv::Size tam = historial_.obtener(q.indice, f) ? f.img.size() : cv::Size(cam_.width, cam_.height);
    q.zona = cuadrado(q.prior, std::max(0.5f * cfg_.margen_local * lado, 32.0f), tam);
    if (enviar(q))
    {
        cont_.pedidos_red++;
        zona_pedida_ = q.zona;
        estado_ = REENGANCHANDO;
        intentos_locales_ = 0;
        cont_.cambios_barrida++;
    }
}
//---------------------------------------------------------------------------
// Descarta el seguidor actual. Se cambia la generacion para que cualquier verificacion
// pendiente sobre el se ignore al llegar.
void PerfilObjeto::soltar()
{
    retadores_vaciar(&ret_);
    kcf_.reset();
    generacion_++;
    visto_ = -1;
    perdidos_ = 0;
    verif_fallidas_ = 0;
}
//---------------------------------------------------------------------------
// Pedido externo de reenganche: se descarta el seguidor y se vuelve a buscar.
void PerfilObjeto::reenganchar(Motivo m)
{
    if (m == Motivo::RECHAZO_KALMAN)
        cont_.reenganches_kalman++;
    soltar();
    estado_ = BUSCANDO;
    intentos_locales_ = 0;
    nuevo_ = false;
}
//---------------------------------------------------------------------------
// Texto del estado actual, para mostrar en pantalla.
const char *PerfilObjeto::estado() const
{
    switch (estado_)
    {
    case SIGUIENDO: return "SIGUIENDO";
    case REENGANCHANDO: return "REENGANCHANDO";
    default: return "BUSCANDO";
    }
}
//---------------------------------------------------------------------------
// Contadores del perfil (enganches, perdidas, verificaciones...), para el resumen.
Contadores PerfilObjeto::contadores() const
{
    return cont_;
}
//---------------------------------------------------------------------------
// Dibuja sobre la imagen lo que esta haciendo el perfil (colores en orden B, G, R):
//  - En gris, la zona que se le pidio a la red, mientras este trabajando.
//  - En azul, la ultima deteccion de la red, con su confianza escrita encima.
//  - La caja del KCF, en rojo si esta perdido, en verde si se esta siguiendo, y en
//    naranjo si se esta reenganchando.
void PerfilObjeto::dibujar(cv::Mat &img) const
{
    if (ocupado() && zona_pedida_.area() > 0)
        cv::rectangle(img, zona_pedida_, cv::Scalar(160, 160, 160), 1);
    if (hay_ultima_det_)
    {
        cv::rectangle(img, ultima_det_.caja, cv::Scalar(255, 128, 0), 1);
        cv::putText(img, cv::format("%.2f", ultima_det_.prob), ultima_det_.caja.tl(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 128, 0), 1);
    }
    if (kcf_)
    {
        const cv::Scalar color = kcf_->lost() ? cv::Scalar(0, 0, 255)
                                 : estado_ == SIGUIENDO ? cv::Scalar(0, 255, 0)
                                                        : cv::Scalar(0, 140, 255);
        cv::rectangle(img, kcf_->roi(), color, 2);
    }
    dibujarRetadores(img, ret_);
}

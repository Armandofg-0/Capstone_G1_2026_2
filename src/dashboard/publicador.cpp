// Puente entre el lazo de control y el servidor web. El lazo no puede esperar a la red, asi
// que solo deja su ultimo cuadro en un "triple buffer" (una caja compartida donde siempre queda
// lo mas reciente) y sigue. Un hilo aparte, el de este archivo, toma de ahi, comprime la
// imagen a JPEG, arma los eventos con los datos y atiende el servidor.
// Publicador del dashboard: ver publicador.hpp.
#include "publicador.hpp"

#include "reloj_pc.h"
#include "ritmo.h"
#include "salud.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

// namespace { ... } sin nombre: lo de adentro solo existe en este archivo (equivale a "static"
// en C), asi sus nombres no chocan con otros archivos.
namespace {

const int ESPERA_MS = 2;          // espera de servidor_atender y duracion minima de una vuelta
const size_t CAP_META = 4096;     // cabe el peor caso de datos_cuadro_json (H16)
const size_t CAP_SALUD = 1024;
const size_t CAP_EVENTO = 8192;

// Copia txt al final de dst (largo n). false si no cabe.
bool agregar(char *dst, size_t cap, size_t &n, const char *txt, size_t k)
{
    if (n + k + 1 > cap) return false;
    std::memcpy(dst + n, txt, k);
    n += k;
    dst[n] = '\0';
    return true;
}

bool agregar(char *dst, size_t cap, size_t &n, const char *txt)
{
    return agregar(dst, cap, n, txt, std::strlen(txt));
}

struct EstadisticasPublicador {
    double fps_video;
    long long descartes, eventos_perdidos;
};

// Las estadisticas del servidor y las del publicador, como objeto JSON. -1 si no cabe.
int servidor_json(const EstadisticasServidor &e, const EstadisticasPublicador &ep, char *dst,
                  size_t cap)
{
    char kernel[24] = "null";
    if (e.cola_kernel_max_bytes >= 0)
        std::snprintf(kernel, sizeof kernel, "%ld", e.cola_kernel_max_bytes);
    const int n = std::snprintf(
        dst, cap,
        "{\"clientes_video\":%d,\"clientes_eventos\":%d,\"cuadros_enviados\":%ld,"
        "\"cuadros_descartados\":%ld,\"eventos_descartados\":%ld,\"cola_max_bytes\":%ld,"
        "\"cola_kernel_max_bytes\":%s,\"fps_video\":%.1f,\"descartes_publicador\":%lld,"
        "\"eventos_perdidos\":%lld}",
        e.clientes_video, e.clientes_eventos, e.cuadros_enviados, e.cuadros_descartados,
        e.eventos_descartados, e.cola_max_bytes, kernel, ep.fps_video, ep.descartes,
        ep.eventos_perdidos);
    return (n < 0 || (size_t)n >= cap) ? -1 : n;
}

// {"cuadro":...,"salud":...,"servidor":...} en dst, sin asignar memoria. d = NULL da "cuadro":null.
// Devuelve el largo, o -1 si no cabe.
int armar_evento(const DatosCuadro *d, const Salud &salud, const EstadisticasServidor &e,
                 const EstadisticasPublicador &ep, char *dst, size_t cap)
{
    size_t n = 0;
    char sal[CAP_SALUD];
    if (!agregar(dst, cap, n, "{\"cuadro\":")) return -1;
    if (d) {
        const int k = datos_cuadro_json(d, dst + n, cap - n);
        if (k < 0) return -1;
        n += (size_t)k;
    } else if (!agregar(dst, cap, n, "null")) {
        return -1;
    }
    const int ks = salud_json(&salud, sal, sizeof sal);
    if (ks < 0 || !agregar(dst, cap, n, ",\"salud\":") || !agregar(dst, cap, n, sal, (size_t)ks))
        return -1;
    if (!agregar(dst, cap, n, ",\"servidor\":")) return -1;
    const int kv = servidor_json(e, ep, dst + n, cap - n);
    if (kv < 0) return -1;
    n += (size_t)kv;
    if (!agregar(dst, cap, n, "}")) return -1;
    return (int)n;
}

}  // namespace

// Constructor: solo guarda los parametros y la pagina; el servidor arranca en iniciar().
Publicador::Publicador(const ParamsDashboard &p, const std::string &pagina)
    : p_(p), pagina_(pagina)
{
}

// Destructor: avisa al hilo que termine (correr_ en false), espera a que salga (join) y
// recien entonces destruye el servidor, que el hilo usaba. correr_ es std::atomic: una
// variable que dos hilos pueden leer y escribir a la vez sin corromperse.
Publicador::~Publicador()
{
    correr_.store(false);
    if (hilo_.joinable()) hilo_.join();
    servidor_destruir(s_);
}

// Valida los parametros, crea el servidor y lanza el hilo que corre bucle(). std::thread
// recibe la funcion miembro y el objeto ("this") sobre el que la llama.
bool Publicador::iniciar()
{
    if (s_) return true;
    if (!(p_.fps_max > 0.0) || !(p_.hz_paneles > 0.0) || p_.calidad_jpeg < 1 ||
        p_.calidad_jpeg > 100) {
        std::fprintf(stderr, "dashboard: fps_max y hz_paneles deben ser > 0 y calidad_jpeg "
                             "estar en 1..100\n");
        return false;
    }
    s_ = servidor_crear(&p_, reinterpret_cast<const uint8_t *>(pagina_.data()), pagina_.size());
    if (!s_) return false;
    if (!sistema_.empty()) servidor_fijar_sistema(s_, sistema_.data(), sistema_.size());
    correr_.store(true);
    hilo_ = std::thread(&Publicador::bucle, this);
    return true;
}

// Lo llama el lazo en cada cuadro: deja imagen y datos en el triple buffer y vuelve al tiro.
// cv::Mat copia solo una referencia a la imagen, no los pixeles, asi que es barato.
void Publicador::publicar(const cv::Mat &img, const DatosCuadro &d)
{
    buf_.escribir(Publicacion{img, d, ++n_publicados_});
}

// El lazo pregunta si la pagina pidio otro nivel. exchange(-1) lee el pedido y lo borra en
// una sola operacion atomica, para no perder ni repetir un pedido entre hilos.
bool Publicador::nivelPedido(int &nivel)
{
    const int n = nivel_pedido_.exchange(-1);
    if (n < 0) return false;
    nivel = n;
    return true;
}

// El lazo informa el nivel vigente. Nivel y bloqueo van empaquetados en un solo entero
// (2 * nivel + bloqueado) para pasarlos juntos en una variable atomica.
void Publicador::fijarNivel(int nivel, bool bloqueado)
{
    nivel_fijado_.store(2 * nivel + (bloqueado ? 1 : 0));
}

// Hilo del publicador. En cada vuelta: (1) toma lo ultimo del lazo, (2) codifica y publica el
// video al ritmo maximo permitido, (3) publica los paneles de datos, (4) atiende los sockets y
// (5) pasa al lazo los pedidos de nivel. Si no hubo nada que hacer, duerme lo que falta.
void Publicador::bucle()
{
    const double periodo_evento = 1000.0 / p_.hz_paneles;
    Ritmo ritmo;
    ritmo_iniciar(&ritmo, 1000.0 / p_.fps_max);
    const std::vector<int> opciones{cv::IMWRITE_JPEG_QUALITY, p_.calidad_jpeg};
    std::vector<uchar> jpeg;
    char meta[CAP_META], evento[CAP_EVENTO];
    Publicacion pub;          // la ultima leida: su imagen se suelta al codificarla
    bool hay_datos = false;   // ya se leyo al menos un cuadro
    bool pendiente = false;   // la imagen de pub aun no se codifica
    long long n_leido = 0;
    EstadisticasPublicador ep = {0.0, 0, 0};
    MedidorSalud medidor;
    Salud salud;
    salud_iniciar(&medidor);
    int nivel_aplicado = -1;
    double t_evento = -1e300, t_ventana = reloj_pc_ms();
    long enviados_ventana = 0;

    while (correr_.load()) {
        const double t = reloj_pc_ms();
        bool trabajo = false;

        const int nf = nivel_fijado_.load();
        if (nf != nivel_aplicado) {
            servidor_fijar_nivel(s_, nf >> 1, nf & 1);
            nivel_aplicado = nf;
        }

        // 1. Lo ultimo del lazo. Lo que el triple buffer sobrescribio sin leer, y el cuadro
        //    pendiente que este reemplaza, son descartes.
        if (buf_.leer(pub)) {
            ep.descartes += pub.n - n_leido - 1 + (pendiente ? 1 : 0);
            n_leido = pub.n;
            hay_datos = pendiente = trabajo = true;
        }

        EstadisticasServidor e;
        servidor_estadisticas(s_, &e);

        // 2. Video: un JPEG por cuadro, a lo mas fps_max por segundo (ritmo.h), solo si alguien
        //    mira.
        if (pendiente && t >= ritmo_turno(&ritmo)) {
            pendiente = false;
            trabajo = true;
            bool ok = false;
            if (e.clientes_video > 0 && !pub.img.empty()) {
                try {
                    ok = cv::imencode(".jpg", pub.img, jpeg, opciones);
                } catch (const cv::Exception &) {
                    ok = false;
                }
            }
            if (ok) {
                int nm = datos_cuadro_json(&pub.d, meta, sizeof meta);
                if (nm < 0) nm = std::snprintf(meta, sizeof meta, "null");
                servidor_publicar_cuadro(s_, meta, (uint32_t)nm, jpeg.data(),
                                         (uint32_t)jpeg.size());
                enviados_ventana++;
                ritmo_enviado(&ritmo, t);
            }
            pub.img.release();  // se suelta aqui y no en el hilo del lazo
        }

        // 3. Paneles: cuadro, salud y estadisticas, cada 1/hz_paneles.
        if (t - t_evento >= periodo_evento) {
            t_evento = t;
            trabajo = true;
            if (t - t_ventana >= 1000.0) {
                ep.fps_video = enviados_ventana * 1000.0 / (t - t_ventana);
                enviados_ventana = 0;
                t_ventana = t;
            }
            salud_leer(&medidor, &salud);
            const int n = armar_evento(hay_datos ? &pub.d : nullptr, salud, e, ep, evento,
                                       sizeof evento);
            if (n > 0)
                servidor_publicar_evento(s_, evento, (size_t)n);
            else
                ep.eventos_perdidos++;
        }

        // 4. Sockets. Con un cuadro esperando su turno de fps_max no se espera mas de lo que
        //    le falta.
        int espera = ESPERA_MS;
        if (pendiente) {
            const double falta = ritmo_turno(&ritmo) - reloj_pc_ms();
            espera = falta <= 0.0 ? 0 : std::min(ESPERA_MS, (int)std::ceil(falta));
        }
        const double t_atender = reloj_pc_ms();
        if (servidor_atender(s_, espera)) trabajo = true;

        // 5. Pedidos de nivel del navegador.
        int nivel;
        if (servidor_nivel_pedido(s_, &nivel)) nivel_pedido_.store(nivel);
        const AccionServidor a = servidor_accion_pedida(s_);
        if (a != ACCION_NINGUNA) accion_.store(a);
        int orden, eje;
        float valor;
        if (servidor_calibracion_pedida(s_, &orden, &eje, &valor)) {
            uint32_t bits;
            std::memcpy(&bits, &valor, sizeof bits);
            calibracion_.store((uint64_t)orden << 40 | (uint64_t)eje << 32 | bits);
        }
        const int cam = servidor_camara_pedida(s_);
        if (cam != CAMARA_NINGUNO) camara_.store(cam);
        int r[4];
        if (servidor_seleccion_pedida(s_, r))
            seleccion_.store((uint64_t)r[0] << 48 | (uint64_t)r[1] << 32 | (uint64_t)r[2] << 16 | (uint64_t)r[3]);

        // Si no hubo nada que hacer (tampoco en los sockets) y servidor_atender volvio antes,
        // se completa la espera: el hilo nunca gira al 100 % de CPU. Con progreso no se
        // duerme: un cuadro de ~200 KB sale en varios trozos de SNDBUF y cada siesta lo
        // atrasaria.
        if (!trabajo) {
            const double resto = espera - (reloj_pc_ms() - t_atender);
            if (resto > 0.0)
                std::this_thread::sleep_for(std::chrono::microseconds((long long)(resto * 1000.0)));
        }
    }
}

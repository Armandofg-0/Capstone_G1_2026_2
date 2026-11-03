// Puente entre el hilo del lazo y el servidor del dashboard.
//
// El hilo del lazo solo llama a publicar, que escribe en un triple buffer: no espera, no
// toma un mutex, no codifica ni toca un socket. Un hilo propio del publicador da vueltas:
// lee el triple buffer, codifica el JPEG una vez (si hay clientes de video y el cuadro tiene
// turno en la grilla de fps_max, ver ritmo.h), publica cada 1/hz_paneles el evento de los
// paneles y atiende al servidor, que es de un solo hilo y vive entero en este hilo.
//
// Evento SSE: {"cuadro":<datos_cuadro_json o null>,"salud":<salud_json>,"servidor":{...}},
// donde "servidor" lleva todos los campos de EstadisticasServidor (cola_kernel_max_bytes es
// null si no esta disponible) mas:
//   - fps_video: cuadros entregados al servidor por segundo, medidos en el publicador;
//   - descartes_publicador: cuadros publicados que no se codificaron porque llego uno mas
//     nuevo (en el triple buffer o mientras esperaban el turno de fps_max);
//   - eventos_perdidos: eventos que no se pudieron armar (no cupieron en el buffer).
#ifndef PUBLICADOR_HPP
#define PUBLICADOR_HPP

#include "datos_cuadro.h"
#include "params.h"
#include "servidor.h"
#include "triple_buffer.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>

struct Publicacion {
    cv::Mat img;
    DatosCuadro d;
    long long n;  // numero de publicacion, lo pone publicar (cuenta los descartes)
};

class Publicador {
public:
    Publicador(const ParamsDashboard &p, const std::string &pagina);
    ~Publicador();  // detiene y une el hilo, y destruye el servidor (haya o no iniciado)
    Publicador(const Publicador &) = delete;
    Publicador &operator=(const Publicador &) = delete;

    // Crea el servidor y lanza el hilo. false si servidor_crear fallo (p. ej. puerto ocupado).
    bool iniciar();

    // Hilo del lazo: solo TripleBuffer::escribir. La imagen se toma por referencia (cabecera de
    // cv::Mat, sin clone). Precondicion: la fuente entrega un cv::Mat nuevo por cuadro y no
    // vuelve a escribir en uno ya publicado, porque el hilo del publicador puede estar
    // codificandolo.
    void publicar(const cv::Mat &img, const DatosCuadro &d);

    // Ultimo nivel pedido desde el navegador (POST /nivel), si hubo uno desde la consulta
    // anterior. Se puede llamar desde cualquier hilo.
    bool nivelPedido(int &nivel);
    // Nivel vigente y candado (bloqueado rechaza los POST con 409). El hilo del publicador lo
    // pasa al servidor en su proxima vuelta. Se puede llamar desde cualquier hilo.
    void fijarNivel(int nivel, bool bloqueado);
    // JSON de GET /sistema. Se llama antes de iniciar (no se cambia despues).
    void fijarSistema(const std::string &json) { sistema_ = json; }
    // Ultima accion pedida por la pagina (AccionServidor), si hubo; la consume. Cualquier hilo.
    int accionPedida() { return accion_.exchange(ACCION_NINGUNA); }
    // Pide una accion desde el propio lazo (la toma accionPedida en el cuadro siguiente).
    void pedirAccion(int a) { accion_.store(a); }
    // Ultimo recuadro pedido por la pagina (POST /seleccion), si hubo; lo consume. Los cuatro
    // valores (de 16 bits) van en un solo atomico para no leer un recuadro a medias.
    // Ultimo comando de calibracion pedido por la pagina (POST /calibracion), si hubo; lo consume.
    bool calibracionPedida(int &orden, int &eje, float &valor)
    {
        const uint64_t v = calibracion_.exchange(0);
        if (!v) return false;
        orden = (int)(v >> 40);
        eje = (int)((v >> 32) & 0xFF);
        const uint32_t bits = (uint32_t)v;
        std::memcpy(&valor, &bits, sizeof valor);
        return true;
    }
    // Ultimo pedido de calibracion de la camara (POST /camara, CAMARA_*); lo consume.
    int camaraPedida() { return camara_.exchange(CAMARA_NINGUNO); }
    bool seleccionPedida(int r[4])
    {
        const uint64_t v = seleccion_.exchange(0);
        if (!v) return false;
        for (int i = 0; i < 4; i++) r[i] = (int)((v >> (16 * (3 - i))) & 0xFFFF);
        return true;
    }

private:
    void bucle();

    ParamsDashboard p_;
    std::string pagina_;
    Servidor *s_ = nullptr;
    TripleBuffer<Publicacion> buf_;
    long long n_publicados_ = 0;           // solo del hilo del lazo
    std::thread hilo_;
    std::atomic<bool> correr_{false};
    std::atomic<int> nivel_pedido_{-1};    // -1: no hay pedido
    std::atomic<int> nivel_fijado_{0};     // 2*nivel + bloqueado
    std::atomic<int> accion_{ACCION_NINGUNA};
    std::atomic<uint64_t> seleccion_{0}; // 0: no hay pedido (ancho y alto nunca son 0)
    std::atomic<uint64_t> calibracion_{0}; // orden (8 bits) | eje (8) | valor (32, float); 0: ninguno
    std::atomic<int> camara_{CAMARA_NINGUNO};
    std::string sistema_;
};

#endif /* PUBLICADOR_HPP */

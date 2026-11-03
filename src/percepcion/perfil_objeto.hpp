// Perfil objeto: para blancos solidos (una pelota, u otra clase de objeto que conozca la
// red neuronal).
//
// Este perfil trabaja con dos hilos, porque la red es demasiado lenta para correr en cada
// cuadro.
//
// El hilo del lazo corre el KCF (un seguidor rapido) en cada cuadro y entrega la
// medicion. Nunca se queda esperando a la red.
//
// El hilo de la red corre el adquisidor sobre un cuadro guardado en el historial. Si encuentra
// el blanco, crea ("siembra") ahi un KCF nuevo, y lo pone al dia con los cuadros que
// llegaron mientras la red trabajaba (en la Raspberry Pi, la red tarda varios periodos).
// Asi, al lazo le llega un seguidor ya enganchado en el presente, y no una caja vieja que
// muestra donde estaba el blanco hace un rato.
//
// Del lado del lazo, el perfil pasa por tres estados:
//
// BUSCANDO: no hay seguidor. Se le piden adquisiciones a la red: primero en una zona
// alrededor de la pista del Kalman (con una entrada chica, que es rapida) y, si ahi no hay
// nada, en toda la imagen.
//
// SIGUIENDO: el KCF corre en cada cuadro. Su ventana se centra en la prediccion del Kalman
// (o, si no hay pista, se traslada con el movimiento propio de la torreta). La sigma de la
// medicion crece cuando el PSR (una medida de que tan seguro esta el KCF) cae respecto de
// su valor habitual. Cada `periodo_verificacion` cuadros, la red revisa un recorte
// alrededor de la caja del KCF: si la caja se corrio un poco, se vuelve a sembrar el KCF
// con la caja de la red; y si la red no ve nada `max_verif_fallidas` veces seguidas, se
// reengancha.
//
// REENGANCHANDO: el KCF sigue corriendo mientras la red busca de nuevo con la ayuda de la
// pista del Kalman. Si el KCF no esta perdido, sus mediciones se siguen entregando, pero
// con la sigma inflada al maximo (se le cree poco).
//
// Hay tres motivos para reengancharse: que el KCF este perdido (segun su PSR) durante
// `max_perdidos` cuadros seguidos, que fallen varias verificaciones, o que lo pida el
// Kalman desde afuera (por rechazar varias mediciones seguidas). En este ultimo caso,
// ademas, se descarta el seguidor actual.
//
// Modo determinista (cuando `latencia_cuadros` > 0, pensado para la simulacion): la
// respuesta de la red se entrega exactamente `latencia_cuadros` cuadros despues del
// cuadro pedido, sin importar cuanto haya tardado de verdad. Asi, una misma corrida se
// repite siempre igual, sin depender de la velocidad del computador.

#pragma once

#include "historial.hpp"
#include "kcftracker.hpp"
#include "percepcion.hpp"
#include "retador.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

// Una deteccion del adquisidor: caja en pixeles de la imagen completa y su confianza.
struct Deteccion
{
    cv::Rect2f caja;
    int clase;  // tipo de blanco (0 = el blanco buscado)
    float prob; // confianza, de 0 a 1
};

// Funcion de deteccion (adquisidor lento). Corre en el hilo de la red: busca el blanco en
// `zona` del cuadro y devuelve cajas en pixeles de la imagen completa. `indice` identifica
// el cuadro (lo usa el oraculo de la simulacion).
//
// std::function permite guardar "cualquier cosa que se pueda llamar como funcion" con esa
// forma: una funcion normal, una lambda, etc. Asi el perfil no depende del adquisidor: en
// la simulacion se le pasa un detector de mentira (el oraculo) que usa la verdad.
typedef std::function<void(const cv::Mat &img, const cv::Rect &zona, int64_t indice,
                           std::vector<Deteccion> &out)>
    FuncionDeteccion;

struct ConfigObjeto
{
    float prob_min = 0.35f;          // confianza minima del adquisidor para aceptar una caja

    float sigma_px = 2.5f;           // sigma de la medicion con el PSR en su referencia
    float sigma_max_factor = 3.0f;   // tope del crecimiento de sigma cuando el PSR cae
    int max_perdidos = 6;            // cuadros seguidos con el KCF perdido antes de reenganchar
    float margen_borde = 0.0f;       // px; una caja mas cerca del borde de la imagen no se mide (0 = sin criterio)

    int periodo_verificacion = 15;   // cuadros entre verificaciones de la red (0 = nunca)
    int max_verif_fallidas = 3;      // verificaciones seguidas sin ver el blanco
    // Desde este IoU, la caja del KCF se da por buena. El IoU ("interseccion sobre
    // union") mide cuanto se superponen dos cajas: 1 si son identicas, 0 si no se tocan.
    float iou_ok = 0.5f;
    float margen_verificacion = 2.0f;// recorte de verificacion = caja del KCF * margen

    float margen_local = 3.0f;       // zona de busqueda local = caja * margen (como minimo
                                     //   el radio de la pista del Kalman)
    int max_intentos_locales = 2;    // despues se busca en toda la imagen
    float radio_min_prior = 30.0f;   // piso del radio de la pista para elegir detecciones

    bool centrar_con_pista = true;   // ventana del KCF en la prediccion del Kalman
    int latencia_cuadros = 0;        // > 0: modo determinista (ver arriba)
    int capacidad_historial = 45;    // cuadros guardados (1.5 s a 30 fps)

    KCFTracker kcf;                  // prototipo: sus parametros se copian a cada seguidor
    ConfigBarrida barrida = barrida_por_defecto(); // busqueda global periodica (retador.h)
};

class PerfilObjeto : public Perfil
{
public:
    // El constructor lanza el hilo de la red, y el destructor lo detiene.
    PerfilObjeto(const Intrinsecos &cam, const ConfigObjeto &cfg, FuncionDeteccion detector);
    ~PerfilObjeto() override;

    Medicion procesar(const Cuadro &c, const Pista &p) override;
    void reenganchar(Motivo m) override;
    const char *estado() const override;
    bool enganchado() const override { return estado_ == SIGUIENDO; }
    void dibujar(cv::Mat &img) const override;
    Contadores contadores() const override;

private:
    enum Estado { BUSCANDO, SIGUIENDO, REENGANCHANDO };
    // Tipos de trabajo para la red: buscar el blanco (ADQUIRIR), revisar que el KCF siga
    // sobre el (VERIFICAR) o buscar retadores en toda la imagen (BARRIDA, retador.h).
    enum Tipo { ADQUIRIR, VERIFICAR, BARRIDA };

    // Trabajo que el hilo del lazo le encarga al hilo de la red.
    struct Pedido
    {
        Tipo tipo = ADQUIRIR;
        int64_t indice = -1;     // cuadro sobre el que corre la red
        cv::Rect zona;           // parte del cuadro que ve la red
        bool local = false;      // zona acotada (para contar intentos)
        bool con_prior = false;  // pista del Kalman para elegir entre detecciones
        cv::Point2f prior;       // centro de esa pista
        float radio_prior = 0.0f;
        cv::Rect2f caja;         // VERIFICAR y BARRIDA: caja del KCF en ese cuadro
        int generacion = 0;      // VERIFICAR y BARRIDA: a que seguidor se refiere
    };

    // Lo que el hilo de la red le devuelve al hilo del lazo.
    struct Respuesta
    {
        Pedido pedido;                      // copia del pedido que se atendio
        bool hay_det = false;               // la red encontro algo
        Deteccion det;
        float iou = 0.0f;                   // VERIFICAR: contra la caja del KCF
        std::vector<Deteccion> todas;       // BARRIDA: detecciones validas, de mayor a menor prob
        std::unique_ptr<KCFTracker> kcf;    // sembrado en det y puesto al dia
        int64_t visto = -1;                 // ultimo cuadro que ya proceso ese KCF
        double ms_red = 0.0;                // tiempo que tardo la red
        double ms_puesta = 0.0;             // computo del KCF nuevo (sin las esperas)
    };

    // --- hilo de la red ---------------------------------------------------------------
    void bucle();
    void atender(const Pedido &p, Respuesta &r);
    std::unique_ptr<KCFTracker> sembrar(const cv::Rect2f &caja, const Cuadro &f,
                                        const Pedido &p, int64_t &visto, double &ms);

    // --- comunicacion ------------------------------------------------------------------
    bool enviar(const Pedido &p);
    bool recoger(int64_t indice_actual, Respuesta &r);
    bool ocupado() const;

    // --- hilo del lazo -----------------------------------------------------------------
    void manejar(Respuesta &r);
    void pedirAdquisicion(const Cuadro &c, const Pista &p);
    void pedirVerificacion(const Cuadro &c);
    void pedirBarrida(const Cuadro &c);
    void manejarBarrida(const Respuesta &r);
    void avanzar(KCFTracker &t, const Cuadro &c, const Pista *p) const;
    void soltar();

    Intrinsecos cam_;
    ConfigObjeto cfg_;
    FuncionDeteccion detector_;
    Historial historial_;

    // Estado del lazo (solo lo toca el hilo del lazo):
    Estado estado_;
    std::unique_ptr<KCFTracker> kcf_;  // seguidor actual (vacio si no hay)
    // Numero que cambia con cada seguidor nuevo. Sirve para descartar respuestas de
    // verificacion que llegan tarde y que se refieren a un seguidor que ya no existe.
    int generacion_;
    int64_t visto_;           // ultimo cuadro procesado por kcf_
    int perdidos_;            // cuadros seguidos con el KCF perdido
    int verif_fallidas_;      // verificaciones seguidas que fallaron
    int intentos_locales_;    // busquedas locales hechas sin exito
    int64_t ultima_verif_;    // indice del cuadro de la ultima verificacion
    int64_t ultima_barrida_;  // indice del cuadro de la ultima barrida global
    Retadores ret_;           // retadores de la barrida global
    bool nuevo_;              // la proxima medicion es la primera de un enganche
    bool hay_caja_;           // ultima caja conocida, para centrar la busqueda local
    cv::Rect2f ultima_caja_;
    Deteccion ultima_det_; // para dibujar
    bool hay_ultima_det_;
    cv::Rect zona_pedida_;    // para dibujar
    Contadores cont_;

    // Comunicacion con el hilo de la red. Todo lo que esta debajo de m_ se lee y escribe
    // solo con m_ tomado (salvo salir_, que es atomico).
    std::thread hilo_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::atomic<bool> salir_;
    // hay_pedido_: hay un pedido esperando a que la red lo tome.
    // en_curso_: hay un pedido enviado cuya respuesta todavia no se recogio.
    // hay_respuesta_: la red termino y dejo su respuesta en respuesta_.
    bool hay_pedido_, en_curso_, hay_respuesta_;
    Pedido pedido_;
    Respuesta respuesta_;
};

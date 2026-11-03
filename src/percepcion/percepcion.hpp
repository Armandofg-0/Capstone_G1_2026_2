// Capa de percepcion de la torreta: toma un cuadro de la camara y lo convierte en una
// medicion del pixel donde esta el blanco, que es lo que necesita el filtro de Kalman del
// control.
//
// Hay un "perfil" por cada tipo de blanco, y los dos se usan de la misma manera (ambos
// heredan de la clase Perfil definida al final de este archivo).
//
// El perfil laser (perfil_laser.hpp) es para blancos puntuales de alto contraste, como el
// punto de un puntero laser. Tanto la deteccion como el seguimiento se hacen con un
// detector CFAR (el SpotTracker), y todo corre en el mismo hilo del lazo de control,
// porque buscar en la imagen completa cuesta apenas alrededor de 1 ms.
//
// El perfil objeto (perfil_objeto.hpp) es para objetos solidos, como una persona o una
// pelota. La adquisicion la hace un detector lento (clasico, sin redes), en un hilo aparte
// porque en la Raspberry Pi tarda varios periodos en procesar un cuadro. Mientras tanto,
// el seguimiento cuadro a cuadro lo hace un KCF (un seguidor mucho mas rapido) en el hilo
// del lazo, y cada cierto tiempo la red verifica que el KCF siga sobre el objeto correcto.
//
// En cada cuadro, el lazo (lazo.hpp) le entrega dos cosas a la percepcion:
//  - El movimiento propio de la torreta desde el cuadro anterior. Con eso, los seguidores
//    trasladan su ventana de busqueda, porque al girar la torreta toda la escena se corre
//    en la imagen (se corre G qd, el mismo termino que usa el Kalman).
//  - La pista del Kalman (el pixel predicho y un radio de incertidumbre). Con eso se
//    elige entre varias detecciones posibles y se acota la zona donde buscar si hay que
//    volver a engancharse.
// Ademas, el lazo le pide a la percepcion que se reenganche cuando el Kalman rechaza
// varias mediciones seguidas. Ese criterio complementa los que tiene cada perfil por su
// cuenta: el PSR del KCF, la verificacion de la red y la perdida del CFAR.

#pragma once

#include "model.h"
#include "retador.h"

#include <opencv2/core.hpp>
#include <cstdint>
#include <vector>

// Movimiento de la torreta en el intervalo entre el cuadro anterior y este.
struct MovimientoPropio
{
    double qd[2] = {0.0, 0.0}; // velocidad media de pan/tilt en el intervalo (rad/s)
    double theta_t = 0.0;      // tilt al empezar el intervalo (rad)
    double Ts = 0.0;           // duracion (s); 0 = sin movimiento conocido
};

// Cuanto se corre en la imagen un punto fijo de la escena que estaba en `pixel`: es el
// termino G qd del Kalman, con G = Ts L_eff evaluado en ese pixel.
cv::Point2f desplazamientoEscena(const Intrinsecos &cam, cv::Point2f pixel, const MovimientoPropio &m);

// true si la caja esta a menos de margen px de algun borde de una imagen de ancho x alto: el
// blanco esta entrando o saliendo del cuadro y el centro de la caja no es el suyo.
bool cajaEnBorde(const cv::Rect2f &caja, int ancho, int alto, float margen);

// Dibuja los retadores de la barrida global: un circulo magenta con sus confirmaciones.
void dibujarRetadores(cv::Mat &img, const Retadores &rs);

// Un cuadro de video junto con la informacion que lo acompana.
struct Cuadro
{
    cv::Mat img;            // BGR (o gris), 8 bits. No se modifica despues de entregarlo:
                            // el hilo de la red lo lee desde el historial.
    int64_t indice = 0;     // numero correlativo del cuadro, sin saltos
    MovimientoPropio mov;   // movimiento de la torreta desde el cuadro anterior hasta este
};

// Lo que sabe el Kalman del blanco en este cuadro (su prediccion a un paso).
struct Pista
{
    bool valida = false;    // false si el Kalman todavia no arranco
    cv::Point2f s;          // pixel predicho
    float radio = 0.0f;     // radio de busqueda (px), del orden de 3 sigma
};

// Lo que la percepcion le entrega al Kalman en cada cuadro.
struct Medicion
{
    bool valida = false;         // false si en este cuadro no se detecto el blanco
    cv::Point2f z;               // pixel medido
    float sigma = 0.0f;          // desviacion de la medicion (px): R = sigma^2 I
    float confianza = 0.0f;      // 0..1, solo informativa
    bool nuevo_enganche = false; // primera medicion de un enganche nuevo: si el Kalman
                                 //   no la puede explicar, se reinicia sobre ella
    cv::Rect2f caja;             // para dibujar
};

// Razon por la que se pide reenganchar.
enum class Motivo
{
    RECHAZO_KALMAN, // el Kalman rechazo varias mediciones seguidas
    MANUAL          // lo pidio el usuario
};

// Estadisticas de funcionamiento de un perfil, para el informe final.
struct Contadores
{
    int enganches = 0;           // adquisiciones que terminaron en seguimiento
    int perdidas = 0;            // perdidas detectadas por el propio perfil
    int reenganches_kalman = 0;  // pedidos por el Kalman
    int pedidos_red = 0;         // trabajos enviados al hilo de la red
    int verificaciones = 0;      // verificaciones de la red que llegaron a evaluarse
    int verif_fallidas = 0;      // verificaciones en las que la red no confirmo al KCF
    int re_siembras = 0;         // el KCF se corrigio con la caja de la red
    int barridas = 0;            // busquedas globales de la barrida con retador
    int cambios_barrida = 0;     // cambios de blanco por un retador confirmado
    std::vector<double> ms_red;          // tiempo de la red por pedido
    std::vector<double> ms_puesta_al_dia; // tiempo de poner al dia un seguidor nuevo
};

// Interfaz comun de los perfiles. Es una clase abstracta: solo dice que funciones debe
// tener un perfil (todas las "= 0"), y cada perfil concreto las implementa a su manera.
// Gracias a esto, el resto del programa puede usar cualquier perfil sin saber cual es.
class Perfil
{
public:
    virtual ~Perfil() {}

    // Procesa el cuadro c con la pista del Kalman para ese cuadro.
    virtual Medicion procesar(const Cuadro &c, const Pista &p) = 0;

    // Criterio externo: soltar el blanco actual y volver a adquirir. La proxima llamada a
    // procesar() ya trae la pista del Kalman para acotar la busqueda.
    virtual void reenganchar(Motivo m) = 0;

    virtual const char *estado() const = 0;        // texto con el estado actual, para mostrar
    virtual bool enganchado() const = 0;           // true si esta siguiendo un blanco
    virtual void dibujar(cv::Mat &img) const = 0;  // dibuja sobre img lo que esta haciendo
    virtual Contadores contadores() const = 0;
};

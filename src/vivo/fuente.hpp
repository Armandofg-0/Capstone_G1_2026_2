// FUENTES DE CUADROS: interfaz de nivel de imagen que pide hal.h. Entrega un cv::Mat, el índice k y
// los instantes del cuadro, todo en el reloj del proceso (reloj_pc_ms). La fuente del sistema es la
// cámara de la Pi (picam) o, si no está, una webcam USB.
//
// Cada cuadro trae un cv::Mat nuevo que la fuente no vuelve a tocar: el publicador del dashboard
// conserva referencias a los cuadros publicados (publicador.hpp).
#pragma once
#include "model.h"
#include "params.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <cstdio>
#include <memory> // Para gestión dinámica de memoria
#include <string>
//--------------------------------------------------------------------------------------------------

// Lecturas vacías seguidas con las que una fuente se da por caída.
const int VACIAS_MAX = 10;

// Cuadro entregado por una fuente, con sus instantes de captura:
struct CuadroFuente {
    cv::Mat img;          // BGR de 8 bits
    long long k;          // Índice del cuadro entregado, desde 0
    double t_llegada_ms;  // Llegada al proceso (reloj_pc_ms)
    double t_sof_ms;      // Inicio de la exposición: SensorTimestamp o t_llegada - latencia_nominal
    bool hay_sensor_ts;   // true si t_sof_ms viene del sensor
};
//--------------------------------------------------------------------------------------------------

// Interfaz común de todas las fuentes de cuadros:
class Fuente {
public:
    virtual ~Fuente() {}
    virtual bool abrir() = 0;                    // false si no se pudo abrir
    virtual bool siguiente(CuadroFuente &c) = 0; // false = falla definitiva (fin del modo vivo)
    virtual const char *nombre() const = 0;      // "picam", "usb" (o "patron" en las pruebas)
};
//--------------------------------------------------------------------------------------------------

// Cámara de la Pi (OV5647) con rpicam-vid: cuadros YUV420 crudos por un pipe, convertidos a BGR.
// No hay SensorTimestamp, entonces t_sof = llegada - latencia_nominal. Solo en Linux.
class FuentePicam : public Fuente {
public:
    // Con archivo no vacio se lee esa grabacion (YUV420 crudo de rpicam-vid, mismo tamano) en
    // vez de lanzar la camara: sirve para repetir una escena con distintas configuraciones.
    FuentePicam(int ancho, int alto, double fps, double latencia_nominal, const std::string &opciones,
                const std::string &archivo = "");
    ~FuentePicam() override;
    bool abrir() override;
    bool siguiente(CuadroFuente &c) override;
    const char *nombre() const override { return "picam"; }

private:
    std::FILE *pipe_ = nullptr;     // Salida estándar de rpicam-vid
    int ancho_, alto_;              // Resolución pedida a la cámara (px)
    double fps_, latencia_nominal_; // Cuadros por segundo y latencia supuesta captura-llegada (s)
    long long k_ = 0;               // Índice del próximo cuadro a entregar
    std::string opciones_;          // Opciones extra de rpicam-vid (exposición, ganancia, AWB)
    std::string archivo_;           // Grabación a reproducir (vacío = cámara)
    double t0_archivo_ms_ = 0.0;    // Instante en que se entregó el primer cuadro de la grabación
};
//--------------------------------------------------------------------------------------------------

// Cámara USB (UVC) con V4L2 a través de OpenCV: YUYV de ancho x alto a fps, con dos búferes.
// Sin SensorTimestamp, como la picam. Solo en Linux.
class FuenteUsb : public Fuente {
public:
    FuenteUsb(const std::string &dispositivo, int ancho, int alto, double fps, double latencia_nominal);
    bool abrir() override; // false también si la cámara no entrega ancho x alto
    bool siguiente(CuadroFuente &c) override;
    const char *nombre() const override { return "usb"; }

private:
    cv::VideoCapture cap_;
    std::string dispositivo_;       // p. ej. /dev/v4l/by-id/usb-...-video-index0
    int ancho_, alto_;              // Resolución pedida (px)
    double fps_, latencia_nominal_; // Cuadros por segundo y latencia supuesta captura-llegada (s)
    long long k_ = 0;               // Índice del próximo cuadro a entregar
};
//--------------------------------------------------------------------------------------------------

// Fuente que corresponde a vivo.camara ("picam" o "usb") según qué cámara hay: auto prefiere la
// picam y cae a la USB; sin ninguna queda la picam, que falla como antes (systemd reintenta).
const char *decidirCamara(int camara, bool hay_picam, bool hay_usb);

// Mira qué cámaras hay y decide la fuente (decidirCamara). En `usb` deja el dispositivo USB:
// vivo.usb si existe o, si está vacío, el primer /dev/v4l/by-id/*-video-index0.
std::string elegirCamara(const ParamsVivo &v, std::string &usb);

// Crea la fuente del modo vivo: la picam, con las opciones de rpicam-vid del perfil, o la USB
// (vivo.usb), que no las usa.
std::unique_ptr<Fuente> crearFuente(const std::string &fuente, const ParamsVivo &v, const std::string &opciones);

// Fuentes de las cámaras (ver fuente.hpp): la de la Pi lanza rpicam-vid y lee sus cuadros por un
// pipe; la USB los toma con V4L2 a través de OpenCV.
#include "fuente.hpp"
#include "reloj_pc.h"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// En Windows, popen y pclose se llaman distinto:
#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#else
#include <glob.h>
#include <unistd.h>
#endif

namespace {

// true si rpicam-hello lista alguna cámara de la Pi (sin ninguna dice "No cameras available!").
bool hayPicam()
{
    std::FILE *p = popen("rpicam-hello --list-cameras 2>&1", "r");
    if (!p) return false;
    std::string salida;
    char buf[256];
    while (std::fgets(buf, sizeof buf, p)) salida += buf;
    pclose(p);
    return salida.find("Available cameras") != std::string::npos;
}

// Dispositivo de la cámara USB: el pedido si existe o, sin pedido, el primer nodo de captura de
// una webcam UVC (by-id no cambia aunque cambie la numeración de /dev/videoN). "" si no hay.
std::string buscarUsb(const std::string &pedido)
{
#ifdef _WIN32
    (void)pedido;
    return "";
#else
    if (!pedido.empty()) return access(pedido.c_str(), F_OK) == 0 ? pedido : "";
    glob_t g;
    std::string r;
    if (glob("/dev/v4l/by-id/*-video-index0", 0, nullptr, &g) == 0 && g.gl_pathc > 0) r = g.gl_pathv[0];
    globfree(&g);
    return r;
#endif
}

} // namespace
//--------------------------------------------------------------------------------------------------
// auto: la picam si está; si no, la USB si está; si no hay ninguna, la picam (falla como antes).
const char *decidirCamara(int camara, bool hay_picam, bool hay_usb)
{
    if (camara == CAMARA_PICAM) return "picam";
    if (camara == CAMARA_USB) return "usb";
    return !hay_picam && hay_usb ? "usb" : "picam";
}
//--------------------------------------------------------------------------------------------------
// Mira las cámaras una sola vez, al arrancar. Una grabación (vivo.archivo) es siempre de la picam.
std::string elegirCamara(const ParamsVivo &v, std::string &usb)
{
    usb = buscarUsb(v.usb);
    if (v.archivo[0]) return "picam";
    const bool hay_picam = v.camara == CAMARA_AUTO && hayPicam();
    const char *f = decidirCamara(v.camara, hay_picam, !usb.empty());
    if (v.camara == CAMARA_AUTO)
        std::printf("vivo: camara de la Pi %s, USB %s: se usa %s\n", hay_picam ? "si" : "no",
                    usb.empty() ? "no" : usb.c_str(), f);
    return f;
}
//--------------------------------------------------------------------------------------------------
// Crea la fuente del modo vivo con los parámetros de [vivo]: la cámara de la Pi o la USB.
std::unique_ptr<Fuente> crearFuente(const std::string &fuente, const ParamsVivo &v, const std::string &opciones)
{
    if (fuente == "usb")
        return std::unique_ptr<Fuente>(new FuenteUsb(v.usb, v.ancho, v.alto, v.fps, v.latencia_nominal));
    return std::unique_ptr<Fuente>(new FuentePicam(v.ancho, v.alto, v.fps, v.latencia_nominal, opciones, v.archivo));
}
//--------------------------------------------------------------------------------------------------
// Constructor: solo guarda la configuración; la cámara no se lanza hasta abrir().
FuentePicam::FuentePicam(int ancho, int alto, double fps, double latencia_nominal, const std::string &opciones,
                         const std::string &archivo)
    : ancho_(ancho), alto_(alto), fps_(fps), latencia_nominal_(latencia_nominal), opciones_(opciones),
      archivo_(archivo)
{
}
//--------------------------------------------------------------------------------------------------
// Destructor: cierra el pipe (y con él rpicam-vid) si estaba abierto.
FuentePicam::~FuentePicam()
{
    if (pipe_) archivo_.empty() ? pclose(pipe_) : std::fclose(pipe_);
}
//--------------------------------------------------------------------------------------------------
// Lanza rpicam-vid sin vista previa ni fin, con las opciones de exposición del perfil, entregando YUV420 por stdout.
bool FuentePicam::abrir()
{
    if (!archivo_.empty()) {
        pipe_ = std::fopen(archivo_.c_str(), "rb");
        return pipe_ != nullptr;
    }
    const std::string cmd = "rpicam-vid -n -t 0 --codec yuv420 --flush --width " + std::to_string(ancho_) +
                            " --height " + std::to_string(alto_) + " --framerate " + std::to_string(fps_) + " " + opciones_ +
                            " -o - 2>/dev/null";
    pipe_ = popen(cmd.c_str(), "r");
    return pipe_ != nullptr;
}
//--------------------------------------------------------------------------------------------------
// Lee el siguiente cuadro del pipe. Un cuadro YUV420 ocupa ancho x alto x 3/2 bytes; si la lectura
// queda corta, la cámara murió y se devuelve false.
bool FuentePicam::siguiente(CuadroFuente &c)
{
    cv::Mat yuv(alto_ * 3 / 2, ancho_, CV_8UC1);
    const size_t n = yuv.total();
    if (!pipe_ || std::fread(yuv.data, 1, n, pipe_) != n) {
        if (k_ == 0) std::fprintf(stderr, "picam: rpicam-vid no entrego cuadros (falta la camara o el programa)\n");
        return false;
    }

    // Una grabación se entrega al ritmo de fps (cuadro k en t0 + k / fps), como la cámara: el EKF
    // usa el reloj para los Delta t.
    if (!archivo_.empty()) {
        if (k_ == 0) t0_archivo_ms_ = reloj_pc_ms();
        const double t = t0_archivo_ms_ + k_ * 1e3 / fps_;
        while (reloj_pc_ms() < t) std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    // Instante de llegada y conversión a BGR (la fuente no entrega el cuadro en YUV):
    c.t_llegada_ms = reloj_pc_ms();
    cv::cvtColor(yuv, c.img, cv::COLOR_YUV2BGR_I420);
    c.k = k_++;

    // Sin SensorTimestamp: el inicio de la exposición se estima restando la latencia nominal.
    c.t_sof_ms = c.t_llegada_ms - latencia_nominal_ * 1e3;
    c.hay_sensor_ts = false;
    return true;
}
//--------------------------------------------------------------------------------------------------
// Constructor: solo guarda la configuración; la cámara no se abre hasta abrir().
FuenteUsb::FuenteUsb(const std::string &dispositivo, int ancho, int alto, double fps, double latencia_nominal)
    : dispositivo_(dispositivo), ancho_(ancho), alto_(alto), fps_(fps), latencia_nominal_(latencia_nominal)
{
}
//--------------------------------------------------------------------------------------------------
// Abre la cámara en YUYV con el tamaño y los fps de [vivo]. Los intrínsecos suponen ese tamaño: si
// la cámara entrega otro, no se usa.
bool FuenteUsb::abrir()
{
    if (dispositivo_.empty() || !cap_.open(dispositivo_, cv::CAP_V4L2)) {
        std::fprintf(stderr, "usb: no se pudo abrir la camara '%s'\n", dispositivo_.c_str());
        return false;
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, ancho_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, alto_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    // Dos búferes: con uno, el driver no tiene dónde escribir mientras se procesa el cuadro y
    // puede perder uno de cada dos. Como el bucle va más rápido que la cámara, no se acumulan.
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);
    const int w = (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH), h = (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (w != ancho_ || h != alto_) {
        std::fprintf(stderr, "usb: la camara entrega %dx%d y se pidio %dx%d\n", w, h, ancho_, alto_);
        return false;
    }
    return true;
}
//--------------------------------------------------------------------------------------------------
// Lee el siguiente cuadro (BGR, en un cv::Mat nuevo). Si la lectura falla, la cámara se
// desconectó y se devuelve false.
bool FuenteUsb::siguiente(CuadroFuente &c)
{
    cv::Mat img;
    if (!cap_.read(img)) {
        if (k_ == 0) std::fprintf(stderr, "usb: la camara '%s' no entrego cuadros\n", dispositivo_.c_str());
        return false;
    }
    c.t_llegada_ms = reloj_pc_ms();
    c.img = img;
    c.k = k_++;

    // Sin SensorTimestamp: el inicio de la exposición se estima restando la latencia nominal.
    c.t_sof_ms = c.t_llegada_ms - latencia_nominal_ * 1e3;
    c.hay_sensor_ts = false;
    return true;
}

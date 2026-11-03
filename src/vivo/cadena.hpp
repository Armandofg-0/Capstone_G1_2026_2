// DEBUG EN VIVO: La idea de este código es facilitar el debuggeo y la validación de distintas etapas,
// entonces, se separa el proyecto en los siguiente niveles:
// Nivel 0: Solo se usa la cámara y el dashboard (validar captura de video y transmisión)  
// Nivel 1: Solo se expone la parte de percepción (validar detección de objetos)
// Nivel 2: Se añade la estimación del EKF (validar la predicción)
// Nivel 3: Se suma la planificación del NMPC (validar la latencia del lazo)
// Nivel 4: Actuación real: Proyecto totalmente funcional. 
#pragma once
#include "adquisidor_varianza.hpp"
#include "config.hpp"
#include "ctl_programado.h"
#include "datos_cuadro.h"
#include "ejecutor_inerte.h"
#include "fuente.hpp"
#include "hal.h"
#include "percepcion.hpp"
#include "interfaz_ctl.h"
#ifdef CON_NMPC
#include "nmpc.h"
#endif
#include <memory> // Para gestión dinámica de memoria

class Cadena {
public:
    Cadena(const Config &cfg, int nivel_inicial, bool bloqueado);
    ~Cadena();
    Cadena(const Cadena &) = delete;
    Cadena &operator=(const Cadena &) = delete;

    // Cambia o consulta el nivel operativo actual (0 a 3)
    void fijarNivel(int nivel);
    int nivel() const { return nivel_; }

    // Nivel 1+: Percepción visual guiada por la pista del EKF
    MedicionCamara percibir(const CuadroFuente &c, const double q[2], DatosCuadro &d);

    // Prepara la estructura de entrada con tiempos, cuadro y telemetría
    void armarEntrada(const MedicionCamara &m, double ahora_ms, EntradaCtl &in);

    // Nivel 2+: Ejecuta el paso del controlador (EKF / NMPC)
    void controlar(const EntradaCtl &in, SalidaCtl &out, DatosCuadro &d);

    // Registra métricas de estimación, planificación y telemetría según el nivel
    void registrar(const EntradaCtl &in, const SalidaCtl &out, DatosCuadro &d);

    // Envía el plan resultante hacia el subsistema inercial
    void enviar(const SalidaCtl &out, double t_ms);

    // Getters de componentes internos
    Controlador interfaz() { return ctl_; }
    const CtlProgramado &ctl() const { return *prog_; }
    const EjecutorInerte &inerte() const { return inerte_; }

    // Nivel 4: abre la UART hacia la ESP32; desde ahi la telemetria y los planes van por ella
    // en vez del ejecutor inerte. Devuelve false si el puerto no abre.
    bool abrirSerie(const char *puerto, int baudios);
    // Nivel 4: manda un comando a la ESP32 por la UART de la cadena (parada o reinicio).
    // Devuelve false si no hay enlace serie.
    bool enviarComando(int orden);
    // Perfil general: recuadro (px del cuadro) que marcó el usuario en el dashboard.
    void seleccionar(int x, int y, int ancho, int alto);
    // Pose articular medida: la de la ultima telemetria de la ESP32 (q0 sin enlace serie).
    const double *qMedida() const { return q_tel_; }
    // Nivel 4: telemetrias recibidas, cuantas traian un plan en ejecucion y su ultima secuencia.
    struct EstadisticaEnlace { long telemetrias = 0, con_plan = 0; int ultima_secuencia = -1; };
    const EstadisticaEnlace &enlace() const { return enlace_; }
#ifdef CON_NMPC
    const Nmpc *nmpc() const { return nmpc_ok_ ? nm_.get() : nullptr; }
#endif

private:
    void reiniciarEstimacion();
    bool pista(Pista &p) const; // Genera el radio de búsqueda usando la covarianza S del EKF

    Config cfg_;
    int nivel_;
    bool bloqueado_, laser_, general_;
    std::unique_ptr<Perfil> perfil_;
    std::shared_ptr<AdquisidorVarianza> varianza_;
    std::unique_ptr<CtlProgramado> prog_;
#ifdef CON_NMPC
    std::unique_ptr<Nmpc> nm_;
#endif
    bool nmpc_ok_ = false;
    Controlador ctl_;
    EjecutorInerte inerte_;
    int fd_serie_ = -1;     // UART hacia la ESP32 (nivel 4), -1 sin enlace
    LectorTrama lector_;    // tramas de telemetria que llegan por la UART
    double q_tel_[2] = {0.0, 0.0};
    EstadisticaEnlace enlace_;
    int k_ = 0; // Índice del cuadro actual
    double q_[2] = {0.0, 0.0}, t_cuadro_ = 0.0; // Pose y tiempo central (rolling shutter)
    double t1_anterior_ = 0.0; // Tiempo de envío del plan anterior
    double t0_tau_ms_; // Origen temporal para medir latencia total
    int rechazos_ = 0; // Conteo de rechazos consecutivos del EKF
};

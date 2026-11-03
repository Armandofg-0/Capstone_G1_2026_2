#include "cadena.hpp"
#include "perfil_laser.hpp"
#include "perfil_general.hpp"
#include "perfil_objeto.hpp"
#include "enlace_serie.h"
#include "mensajes.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// NOTA: en el código se utilizan condicionales que permiten que todo funcione incluso si no compiló el
// código del NMPC (porque requiere acados)

// Verificar M y horizonte MPC:
#ifdef CON_NMPC
static_assert(DC_HORIZONTE == NMPC_HORIZONTE + 1 && DC_M_FOV == M_FOV, "DatosCuadro y el NMPC no coinciden");
#endif
//--------------------------------------------------------------------------------------------------

namespace { // Función solo de uso interno (para que no choque con otro archivo luego)
// Tipo de actuación según el nivel: 4 actúa; 3 planifica; 2, 1 y 0 estiman. 
ActuacionCtl actuacion_de(int nivel)
{
    return nivel >= 4 ? CTL_ACTUAR : (nivel == 3 ? CTL_SOMBRA : CTL_SOLO_ESTIMAR);
}
}
//--------------------------------------------------------------------------------------------------
// Creamos la clase "Cadena" e inicializamos valores en ella (cfg, nivel, bloqueado, laser, prog, tau).
// En esta función se inicializan los perfiles "laser", "general" y "objeto"
Cadena::Cadena(const Config &cfg, int nivel_inicial, bool bloqueado)
    : cfg_(cfg), nivel_(std::min(4, std::max(0, nivel_inicial))), bloqueado_(bloqueado),
      laser_(cfg.perfil == "laser"), general_(cfg.perfil == "general"), prog_(new CtlProgramado),
      t0_tau_ms_(NAN)
{
    const Intrinsecos &cam = cfg_.p.cam; // parámetros de la calibración
    if (laser_)
        perfil_.reset(new PerfilLaser(cam, cfg_.laser));
    else if (general_)
        perfil_.reset(new PerfilGeneral(cam, cfg_.general)); // KCF sobre el recuadro del usuario
    else {
        // Adquisición por color: el perfil objeto recibe el mapa de rojo (ver percibir) y el
        // detector lo umbraliza sobre el cuadro que le toque del historial. La varianza temporal
        // (registrada en percibir con la pose de cada cuadro) desempata contra el rojo quieto.
        varianza_.reset(new AdquisidorVarianza(cam, AdquisidorVarianza::Config()));
        std::shared_ptr<AdquisidorVarianza> v = varianza_;
        const ConfigRojo rojo = cfg_.rojo;
        FuncionDeteccion det = [v, rojo](const cv::Mat &mapa, const cv::Rect &zona, int64_t k, std::vector<Deteccion> &out) {
            cv::Mat mov;
            v->movimiento(k, mov);
            detectarRojo(mapa, zona, rojo, mov, out);
        };
        perfil_.reset(new PerfilObjeto(cam, cfg_.objeto, det)); // PerfilObjeto recibe el detector.
    }
//--------------------------------------------------------------------------------------------------
// Se crea el solver:
#ifdef CON_NMPC
    nm_.reset(new Nmpc());
    nmpc_ok_ = nmpc_iniciar(nm_.get(), &cfg_.p) != 0;
    if (!nmpc_ok_)
        std::fprintf(stderr, "vivo: no se pudo crear el solver del NMPC; el nivel 3 no planifica\n");
#endif
    reiniciarEstimacion();
    ctl_ = ctl_programado_interfaz(prog_.get());
}
// Destructor: libera recursos cuando "Cadena" deja de existir.
Cadena::~Cadena()
{
    serie_cerrar(fd_serie_);
#ifdef CON_NMPC
    if (nmpc_ok_)
        nmpc_liberar(nm_.get());
#endif
}
//--------------------------------------------------------------------------------------------------
// Vuelve la estimacion y el control a cero: EKF, sincronizacion, NMPC y el ejecutor inerte.
void Cadena::reiniciarEstimacion()
{
    ctl_programado_iniciar(prog_.get(), &cfg_.p);  // EKF, Phi, sincronizacion y ekf_estaba en cero
// Registro del NMPC como planificador y se reinicia:
    #ifdef CON_NMPC
    if (nmpc_ok_) {
        ctl_programado_usar_planificador(prog_.get(), nmpc_planificar, nmpc_planificador_reiniciar, nm_.get());
        nmpc_reiniciar(nm_.get());
    }
#endif
    // Se fija el tipo de actuación según el nivel escogido:
    ctl_programado_fijar_actuacion(prog_.get(), actuacion_de(nivel_));

    inerte_iniciar(&inerte_, cfg_.p.vivo.q0); // Esto emula a la ESP32 cuando no hay comunicación serial (0-3)
    trama_iniciar(&lector_);
    q_tel_[0] = cfg_.p.vivo.q0[0];
    q_tel_[1] = cfg_.p.vivo.q0[1];
    t1_anterior_ = 0.0;
    t0_tau_ms_ = NAN;
    rechazos_ = 0;
}
//--------------------------------------------------------------------------------------------------
// Nivel 4: abre la UART hacia la ESP32 (cierra la anterior si la habia).
bool Cadena::abrirSerie(const char *puerto, int baudios)
{
    serie_cerrar(fd_serie_);
    fd_serie_ = serie_abrir(puerto, baudios);
    return fd_serie_ >= 0;
}
//--------------------------------------------------------------------------------------------------
// Cambia el nivel en caliente (desde el dashboard, solo entre 0 y 3). Al bajar de 2 se
// reinicia la estimacion; en cualquier caso se ajusta lo que se hace con el plan.
void Cadena::fijarNivel(int nivel)
{
    if (bloqueado_ || nivel < 0 || nivel > 3 || nivel == nivel_)
        return;
    const int antes = nivel_;
    nivel_ = nivel;
    if (antes >= 2 && nivel < 2)
        reiniciarEstimacion(); // Solo ocurre si baja de nivel
    // Esta fijación permite actualizar la actuación sin cambiar la estimación:
    ctl_programado_fijar_actuacion(prog_.get(), actuacion_de(nivel_));

    // En CTL_SOLO_ESTIMAR ctl_programado no actualiza ekf_estaba: al entrar al nivel 3 la
    // solucion anterior del planificador (si la hay) es de otro enganche. Se olvida.
    if (nivel == 3) {
        prog_->ekf_estaba = 0;
#ifdef CON_NMPC
        if (nmpc_ok_)
            nmpc_reiniciar(nm_.get());
#endif
    }
}
//--------------------------------------------------------------------------------------------------
// Genera una pista de búsqueda visual basada en el Filtro de Kalman (EKF) 
// para el cuadro actual, calculando la posición estimada y un radio de incertidumbre.
bool Cadena::pista(Pista &p) const // struct pista: "p"
{
    double s[2], S[4]; // s: posición predicha en imagen; S: matriz de covarianza (2x2)
    
    // Validaciones previas:
    // 1. nivel_ >= 2 (se requiere estimación activa).
    // 2. El EKF debe estar inicializado.
    // 3. ekf_puerta_imagen proyecta el estado del EKF a la imagen y calcula la covarianza S.
    if (nivel_ < 2 || !prog_->ekf.inicializado
        || !ekf_puerta_imagen(&prog_->ekf, &prog_->geo, q_, t_cuadro_, prog_->sigma_ultima, s, S))
        return false;
    
    // Cálculo del valor porpio máximo (lmax) de la matriz de covarianza de la innovación "S".
    // S se compone como [a, b; b, d] (donde S[0]=a, S[1]=S[2]=b, S[3]=d).
    const double a = S[0], b = 0.5 * (S[1] + S[2]), d = S[3];
    const double lmax = 0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b);
    
    // Construcción y retorno de la estructura de pista:
    p.valida = true;
    p.s = cv::Point2f((float)s[0], (float)s[1]); // Coordenadas estimadas en el plano de la imagen
    
    // El radio se calcula multiplicando el factor de confianza (k_sigma) por la raíz del valor propio máximo,
    // asegurando por seguridad que nunca sea inferior al umbral mínimo permitido (radio_min_pista).
    p.radio = (float)std::max(cfg_.p.lazo.radio_min_pista, cfg_.p.lazo.k_sigma_pista * std::sqrt(lmax));
    
    return true;
}
//--------------------------------------------------------------------------------------------------
// En esta función se define el manejo de los tiempos dado el "rolling shutter" (la imagen se captura
// fila a fila de arriba hacia abajo), la sincronización y la forma de procesar la imagen según el perfil
// escogido de trabajo:
MedicionCamara Cadena::percibir(const CuadroFuente &c, const double q[2], DatosCuadro &d)
{
    const ParamsCamara &pc = cfg_.p.camara;
    MedicionCamara m;
    
    // Conversión de tiempos de la cámara a microsegundos para la estructura de medición:
    // t_sof_ms: Inicio del cuadro (Start of Frame), t_fila: tiempo por línea, t_e: tiempo de exposición.
    m.t_sof_us = (int64_t)std::llround(c.t_sof_ms * 1e3);
    m.t_fila_us = pc.t_fila * 1e6;
    m.t_e_us = pc.t_e * 1e6;
    
    // Valores predeterminados en ausencia de detección válida.
    m.hay_deteccion = 0;
    m.z[0] = m.z[1] = m.sigma = m.tamano = NAN;
    
    // Almacena el índice del cuadro actual y la posición de los ejes (q).
    k_ = (int)c.k;
    q_[0] = q[0];
    q_[1] = q[1];
    
    // Cálculo preciso del instante temporal del centro del cuadro considerando el efecto 
    // del "rolling shutter": Inicio del cuadro + mitad de la altura de la imagen 
    // por el tiempo de fila + la mitad del tiempo de exposición.
    t_cuadro_ = c.t_sof_ms * 1e-3 + 0.5 * cfg_.p.cam.height * pc.t_fila + 0.5 * pc.t_e;
    
    // Copia "metadatos" generales del perfil y del centro óptico en la estructura de telemetría (DatosCuadro).
    std::snprintf(d.perfil, sizeof d.perfil, "%s", cfg_.perfil.c_str());
    d.centro[0] = d.s_obj[0] = cfg_.p.cam.cu;
    d.centro[1] = d.s_obj[1] = cfg_.p.cam.cv;
    
    // Si el nivel operativo es menor a 1, la capa de percepción está inactiva y se retorna la estructura vacía.
    if (nivel_ < 1)
        return m;

    // Si el adquisidor de varianza está activo, registra el cuadro y la posición actual de los ejes.
    if (varianza_)
        varianza_->registrar(c.k, c.img, q);
        
    // Prepara la estructura del cuadro de imagen para el procesamiento.
    Cuadro cu;
    if (laser_ || general_)
        cu.img = c.img;
    else
        mapaRojo(c.img, cu.img); // el perfil objeto (KCF y adquisidor) trabaja sobre el mapa de rojo
    cu.indice = c.k;  // Por simplicidad: Se asume que los ejes no se mueven durante los us de exposición.
    
    // Genera la pista predictiva (ventana de búsqueda basada en el EKF).
    Pista p;
    pista(p); // función de antes
    
    // Procesa la imagen mediante el perfil correspondiente (PerfilLaser o PerfilObjeto) acotado por la pista.
    const Medicion med = perfil_->procesar(cu, p);

    // Registra el estado actual del perfil y verifica si la medición fue exitosa.
    std::snprintf(d.estado_perfil, sizeof d.estado_perfil, "%s", perfil_->estado());
    d.hay_z = med.valida ? 1 : 0;
    
    cv::Rect2f v;
    if (laser_)
        v = cv::Rect2f(static_cast<const PerfilLaser *>(perfil_.get())->seguidor().ventana());
    else if (med.valida)
        v = med.caja;
        
    // Si se obtuvo una ventana o caja delimitadora válida, se guardan sus coordenadas y dimensiones.
    if (v.area() > 0.0f) {
        d.ventana[0] = v.x;
        d.ventana[1] = v.y;
        d.ventana[2] = v.width;
        d.ventana[3] = v.height;
    }
    
    // Si la medición del perfil es válida, se extraen las coordenadas detectadas (z), 
    // la desviación estándar (sigma), la confianza y el tamaño aparente (si aplica).
    if (med.valida) {
        d.z[0] = m.z[0] = med.z.x;
        d.z[1] = m.z[1] = med.z.y;
        d.sigma = m.sigma = med.sigma;
        d.confianza = med.confianza;
        m.hay_deteccion = 1;
        
        // El tamaño aparente (envergadura en píxeles) se calcula exclusivamente para el perfil de objeto.
        if (!laser_ && !general_)
            m.tamano = med.caja.width;
    }
    
    return m;
}
//--------------------------------------------------------------------------------------------------
// Función de gran importancia: Se arma la entrada del controlador. Esta incluye la medición de la cámara 
// (detección, sigma, tamaño), la telemetría del ejecutor inerte (la ESP32 en la aplicación final) y la
// sincronización para el retardo. 
void Cadena::armarEntrada(const MedicionCamara &m, double ahora_ms, EntradaCtl &in)
{
    // Limpia toda la estructura de entrada para evitar basura en memoria
    std::memset(&in, 0, sizeof in);
    
    // Asigna el índice del cuadro actual
    in.k = k_;
    
    // Define los tiempos actuales del sistema en segundos
    in.t_s = in.t_s_pi = ahora_ms * 1e-3;
    
    // Copia los datos de la detección visual (validez, coordenadas, sigma y tamaño)
    in.hay_z = m.hay_deteccion;
    in.z[0] = m.z[0];
    in.z[1] = m.z[1];
    in.sigma = m.sigma;
    in.tamano = m.tamano;
    
    // Copia los parámetros temporales de la cámara (rolling shutter) en microsegundos
    in.t_sof_us = m.t_sof_us;
    in.t_fila_us = m.t_fila_us;
    in.t_e_us = m.t_e_us;
    
    // Inicializa el tiempo de referencia base (t0) en el primer ciclo si aún es NaN
    if (std::isnan(t0_tau_ms_))
        t0_tau_ms_ = ahora_ms;
        
    if (fd_serie_ < 0) {
        // Sin enlace serie: telemetría del ejecutor inerte, basada en el tiempo transcurrido
        in.n_msg = 1;
        inerte_telemetria(&inerte_, (uint32_t)(int64_t)std::llround((ahora_ms - t0_tau_ms_) * 1e3), in.msg[0]);
        in.msg_t4[0] = in.t_s_pi;
    } else {
        // Nivel 4: todas las telemetrías que llegaron por la UART desde el cuadro anterior; t4 es
        // el instante de esta lectura (la tarea no lee la UART entre cuadros).
        uint8_t buf[4096];
        int n;
        while ((n = serie_leer(fd_serie_, buf, sizeof buf)) > 0) {
            for (int i = 0; i < n; ++i) {
                const int largo = trama_agregar(&lector_, buf[i]);
                MsgTelemetria t;
                if (largo != MSG_TELEMETRIA_BYTES || !msg_telemetria_decodificar(lector_.salida, largo, &t))
                    continue;
                q_tel_[0] = t.q_hat[0];
                q_tel_[1] = t.q_hat[1];
                enlace_.telemetrias++;
                if (!(t.banderas & TEL_SIN_PLAN)) {
                    enlace_.con_plan++;
                    enlace_.ultima_secuencia = t.secuencia;
                }
                if (in.n_msg == TEL_MAX) // se conservan las más recientes
                    std::memmove(in.msg[0], in.msg[1], sizeof in.msg[0] * (TEL_MAX - 1));
                else
                    in.n_msg++;
                std::memcpy(in.msg[in.n_msg - 1], lector_.salida, MSG_TELEMETRIA_BYTES);
                in.msg_t4[in.n_msg - 1] = in.t_s_pi;
            }
        }
    }

    // Guarda el instante del plan anterior
    in.t1_plan_anterior = t1_anterior_;
}
//--------------------------------------------------------------------------------------------------
// Se ejecuta el controlador si corresponde y se extraen las métricas que este entrega:
void Cadena::controlar(const EntradaCtl &in, SalidaCtl &out, DatosCuadro &d)
{
    if (nivel_ < 2)
        return;
    std::memset(&out, 0, sizeof out);
    ctl_.paso(ctl_.ctx, &in, &out); // Ejecuta el controlador y convierte "In" en "Out"
    registrar(in, out, d); // Extrae las métricas de los resultados (NIS, FOV, estado)
}
//--------------------------------------------------------------------------------------------------
// Función que define la información registrada según el nivel en el que nos encontremos:
void Cadena::registrar(const EntradaCtl &in, const SalidaCtl &out, DatosCuadro &d)
{
    // Si el nivel operativo es menor a 2, la capa de estimación no está activa y se sale.
    if (nivel_ < 2)
        return;
    const EKFPieza &ekf = prog_->ekf;

    // NIVEL 2: Estimación y Filtrado (EKF)
    d.ekf_activo = ekf.inicializado;
    d.aceptada = out.aceptada;
    d.nis = out.nis;
    // Dimensión del NIS: 3 si se usa el tamaño y hay datos válidos, de lo contrario es 2.
    d.dim_nis = ekf.cfg.usar_tamano && std::isfinite(in.tamano) ? 3 : 2;
    d.nis_ref = d.dim_nis == 3 ? EKF_NIS_REF_3 : EKF_NIS_REF_2;
    d.chi2_puerta = d.dim_nis == 3 ? EKF_CHI2_3 : EKF_CHI2_2;
    
    if (ekf.inicializado) {
        // Proyecta el estado actual del EKF a la imagen (posición estimada s_hat y covarianza S).
        if (!ekf_puerta_imagen(&ekf, &prog_->geo, q_, t_cuadro_, prog_->sigma_ultima, d.s_hat, d.S))
            d.s_hat[0] = d.s_hat[1] = d.S[0] = d.S[1] = d.S[2] = d.S[3] = NAN;
        d.rho = ekf.xi[6]; // Parámetro de profundidad inversa (o distancia estimada)
        d.phi_k = ekf.phi_k;
        d.fallos_ekf = ekf.fallos;
        d.modo_ekf = ekf.modo;
        
        // Si el filtro usa IMM (Interactive Multiple Model), registra las probabilidades de cada modo.
        if (ekf.cfg.seleccion == SELECCION_IMM) {
            d.mu[0] = ekf.mu[0];
            d.mu[1] = ekf.mu[1];
        }
    }
    
    // Gestión de rechazos consecutivos por el filtro de Kalman:
    // Si hay medición pero el filtro la rechaza, acumula rechazos hasta forzar un reenganche del perfil.
    if (in.hay_z) {
        if (out.aceptada)
            rechazos_ = 0;
        else if (++rechazos_ >= cfg_.p.lazo.max_rechazos) {
            perfil_->reenganchar(Motivo::RECHAZO_KALMAN);
            rechazos_ = 0;
        }
    }
    d.ge_p = out.ge_p;   // Métrica geométrica (mapa de mirada)
    d.margen = out.margen;
    
    // Si el nivel es menor a 3, termina aquí (modo solo estimación).
    if (nivel_ < 3)
        return;

    // NIVEL 3: Control, Planificación y Modo Sombra
    d.estado_qp = out.estado_qp;
    d.holgura = out.holgura;
    d.qp_iter = out.qp_iter;
    d.ms_qp = out.ms_qp;
    d.bytes_plan = out.n_bytes;
    d.delta_fb_ms = cfg_.p.com.delta_fb * 1e3;
    
    // Calcula la diferencia temporal de la predicción si está disponible.
    if (out.hay_prediccion)
        d.ta_menos_ts_ms = (sync_t_de_tau(&prog_->sync, (double)out.tau_a) - in.t_s_pi) * 1e3;
    // Básicamente: calcula el tiempo restante en ms hasta que el plan se ejecute en la esp32. 

#ifdef CON_NMPC
    // Si el NMPC está activo y el planificador corrió en este ciclo con un EKF válido:
    if (nmpc_ok_ && out.hay_prediccion && ekf.inicializado) {
        const Nmpc &n = *nm_;
        d.ms_rti = n.ms;
        d.fov_r = n.r_fov;
        for (int j = 0; j < DC_M_FOV; ++j)
            d.fov_b[j] = n.fov_b[j];
            
        double r[NMPC_HORIZONTE + 1][2];
        d.n_horizonte = nmpc_horizonte_rumbo(&n, r);
        
        // Transforma los rumbos relativos del NMPC a coordenadas absolutas en píxeles de la cámara.
        for (int i = 0; i < d.n_horizonte; ++i) {
            d.horizonte[i][0] = cfg_.p.cam.cu + r[i][0];  
            d.horizonte[i][1] = cfg_.p.cam.cv + r[i][1];
        }
    }
#endif

    // Si el nivel es menor a 4, termina aquí (modo sombra / planificación sin actuación directa).
    if (nivel_ < 4)
        return;

    // NIVEL 4: Actuación Completa y Telemetría
    d.limitador = d.frenando = d.banderas_esp32 = 0;
    
    // Decodifica los mensajes de telemetría provenientes del ejecutor físico.
    for (int i = 0; i < in.n_msg; ++i) {
        MsgTelemetria t;
        if (!msg_telemetria_decodificar(in.msg[i], MSG_TELEMETRIA_BYTES, &t))
            continue;
        d.limitador |= (t.banderas & TEL_LIMITADOR) != 0;
        d.frenando |= (t.banderas & TEL_FRENANDO) != 0;
        d.banderas_esp32 |= t.banderas;
        d.edad_buffer_ms = t.edad * cfg_.p.lazo.Ts * 1e3; // Antigüedad del buffer en ms
    }
}
//--------------------------------------------------------------------------------------------------
// Proceso de envío de la información al ejecutor. 
void Cadena::enviar(const SalidaCtl &out, double t_ms)
{
    // Si no hay bytes de salida para transmitir, la función se detiene.
    if (out.n_bytes <= 0)
        return;
        
    // Los bytes del plan van a la ESP32 por la UART (nivel 4) o al ejecutor inerte (0-3).
    if (fd_serie_ >= 0)
        serie_escribir(fd_serie_, out.bytes, out.n_bytes);
    else
        inerte_recibir(&inerte_, out.bytes, out.n_bytes);
    
    // Actualiza el registro temporal del último plan enviado, convirtiendo el tiempo de milisegundos a segundos.
    t1_anterior_ = t_ms * 1e-3;
}
//--------------------------------------------------------------------------------------------------
// Recuadro marcado en el dashboard (perfil general): se siembra en el próximo cuadro. Con otro
// perfil no hace nada.
void Cadena::seleccionar(int x, int y, int ancho, int alto)
{
    if (general_)
        static_cast<PerfilGeneral *>(perfil_.get())->seleccionar(cv::Rect(x, y, ancho, alto));
}
//--------------------------------------------------------------------------------------------------
// Comando a la ESP32 por la UART del nivel 4 (la parada de emergencia y el reinicio valen en
// cualquier nivel).
bool Cadena::enviarComando(int orden)
{
    if (fd_serie_ < 0)
        return false;
    const MsgComando c = {(uint8_t)orden, 0, 0.0f};
    uint8_t m[MSG_COMANDO_BYTES], t[2 * MSG_COMANDO_BYTES + 2];
    msg_comando_codificar(&c, m);
    serie_escribir(fd_serie_, t, trama_codificar(m, MSG_COMANDO_BYTES, t));
    return true;
}

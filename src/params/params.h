// Parametros centralizados: el unico lugar con valores por defecto de todo lo que corre en
// C (camara, lazo, EKF, NMPC, comunicacion). Los modulos reciben sus structs ya llenos y no
// tienen defaults propios.
//
// Uso: p = params_por_defecto(); despues params_leer(&ini, &p) aplica lo que diga el INI y
// la linea de comandos (ver ini.h y config/torreta.ini, donde esta explicada cada clave).
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

#include "ekf_pieza.h"
#include "ini.h"
#include "model.h"

#ifdef __cplusplus
extern "C" {
#endif

// [lazo]
typedef struct {
    double Ts;             // periodo del lazo (s); en el INI se escribe fps = 1/Ts
    int max_rechazos;      // mediciones seguidas rechazadas antes de pedir reenganche
    double k_sigma_pista;  // radio de la pista = k * sigma de la innovacion
    double radio_min_pista; // px
} ParamsLazo;

// [camara] (ademas de los intrinsecos).
typedef struct {
    double lx;           // l_x: offset del centro optico respecto del eje de pan (m)
    double t_e;          // exposicion (s)
    double t_fila;       // lectura de una fila, obturador rodante (s)
    double sigma0;       // ruido base del detector (px)
    double kw, bw, sigma_w; // tamano aparente h_w = kw rho/a_z + bw + N(0, sigma_w^2) (px)
} ParamsCamara;

// [tiempos]: cronologia del ciclo (perdida.tex, "Cronologia del ciclo").
typedef struct {
    double baudios;      // UART
    double f_telemetria; // frecuencia de la telemetria ESP32 -> Pi (Hz)
} ParamsTiempos;

// [planta]: ejes con stepper; indice 0 = pan, 1 = tilt.
typedef struct {
    double J[2];          // inercia total por eje: rotor + carga (kg m^2)
    double b[2];          // roce viscoso (N m s/rad)
    double tau_c[2];      // roce de Coulomb (N m)
    double Km_Ip;         // torque en micropasos K_m I_p (N m)
    int Nr;               // dientes del rotor
    double Td;            // torque de detencion (N m)
    double mgl;           // carga gravitatoria del tilt m g l_cg (N m)
    int micropasos[2];    // mu [ABIERTA] por eje (pan, tilt): 1, 2, 4 u 8
    double relacion[2];   // vueltas del motor por vuelta del eje (3 con la polea 20/60)
    int cuentas_encoder;  // cuentas por vuelta del encoder
    double dt_ejecutor;   // periodo del ejecutor de la ESP32 (s)
    int tilt_activo;      // 0: el tilt no tiene motor y queda fijo en tilt_fijo
    double tilt_fijo;     // angulo del tilt cuando no esta activo (rad)
} ParamsPlanta;

// [comunicacion]: aplicacion programada y enlace (perdida.tex). Las [ABIERTA] (Delta_fb, M,
// N_ff, duplicacion, UART) quedan aqui como parametros.
typedef struct {
    double delta_fb;      // Delta_fb: t_a = t_s + Delta_fb (s) [ABIERTA]
    int M;                // nodos optimos enviados [ABIERTA] (<= N del controlador)
    int N_ff;             // nodos de la cola de feedforward [ABIERTA]
} ParamsComunicacion;

// [ejecutor]: nucleo del firmware (src/firmware/ejecutor.h).
typedef struct {
    double a_lim;         // limitador (rad/s^2): sobre el q_ddot del planificador (ver params.c)
    double a_frenado;     // frenado tras la cola y en los limites (rad/s^2)
    double margen_limite; // distancia a los limites de recorrido (rad)
    double kp;            // correccion proporcional del DDA
    double tau_correccion; // filtro del error de esa correccion (s)
    double alfa, beta;    // observador alfa-beta
    double qd_max;        // tope fisico de velocidad de la torreta (rad/s)
} ParamsEjecutor;

// [limites]: recorrido de los ejes, FOV y aceleracion de la busqueda.
typedef struct {
    double q_min[2], q_max[2]; // recorrido permitido [pan, tilt] (rad)
    double r_fov;              // radio de la FOV util, centrado en la imagen (px)
    double qdd_max;            // aceleracion de la busqueda sin blanco (rad/s^2)
} ParamsLimites;

// [nmpc]: NMPC SQP-RTI del informe (src/control/nmpc.h, codegen/nmpc.py).
typedef enum { QP_MODO_BALANCE = 0, QP_MODO_SPEED, QP_MODO_SPEED_ABS, QP_MODO_ROBUST, QP_MODO_N } QpModo;
typedef struct {
    double W_s;          // peso del error de imagen (px^-^2): la escala del costo
    double w_p;          // precio de duplicar la varianza de la medicion por desenfoque
    double W_u, W_delta; // correccion respecto del feedforward y su salto ((rad/s^2)^-^2)
    double eps_P;        // regularizacion del error de velocidad en Q_P (terminal)
    double eps_fov;      // probabilidad de salida por lado: beta_FOV = Phi^-1(1 - eps_FOV)
    double b_max;        // tope del retranqueo de la FOV (px)
    double eta;          // margen sobre el torque de pull-out (0.5 a 0.7)
    double u_max;        // cota de seguridad de q_ddot, bajo el limitador del ejecutor (rad/s^2)
    int iter_arranque;   // iteraciones SQP al arrancar o reenganchar
    int confianza_a;     // encoger psi_ddot_hat por su relacion senal a ruido antes de predecir (1 = si)
    int confianza_v;     // canal de psi_dot_hat: 0 no; 1 por P_psi; 2 por su relacion senal a ruido (nmpc.c)
    // QP de HPIPM, en tiempo de ejecucion (sin regenerar el solver; fase 6, H32).
    int qp_iter_max;     // tope de iteraciones; en SQP-RTI, la solucion con el tope se acepta
    int qp_modo;         // QpModo: compromiso velocidad/robustez de HPIPM
    int qp_arranque_caliente; // 0 no; 1 primales; 2 primales y duales
    int qp_cond_N;       // bloques de la condensacion parcial (N = sin condensar)
} ParamsNmpc;

// [dashboard]: monitor web servido por la Pi.
typedef enum { VIDEO_MJPEG = 0, VIDEO_H264 } FormatoVideo; // H264 reservado, sin implementar
typedef struct {
    int habilitado;       // levantar el servidor
    int puerto;
    char escucha[32];     // direccion de escucha; 127.0.0.1 = solo la maquina local
    int clientes_max;     // pestanas simultaneas: tope de streams de video y, aparte, de
                          // eventos (una pestana abre uno de cada uno); el sobrante recibe 503
    FormatoVideo formato;
    int calidad_jpeg;     // 1..100
    double fps_max;       // tope de cuadros de video por segundo
    double hz_paneles;    // frecuencia de los paneles de datos (Hz)
    char ajustes[256];    // archivo que escribe POST /ajustes; lo fija main (config/ajustes.ini)
} ParamsDashboard;

// [vivo]: modo vivo (lazo sobre una fuente de video en tiempo real). `nivel` es el nivel de
// operacion (0..4), no el `general.modo`.
typedef enum { CAMARA_AUTO = 0, CAMARA_PICAM, CAMARA_USB } CamaraVivo; // vivo.camara
typedef struct {
    int nivel;              // 0..4
    int ancho, alto;        // px
    double fps;
    double q0[2];           // pose inicial de la torreta, [pan, tilt] (rad)
    double latencia_nominal; // latencia de captura supuesta (s)
    double segundos;        // duracion; 0 = sin fin
    int tiempo_real;
    char puerto[64];        // UART hacia la ESP32 (nivel 4)
    char archivo[256];      // grabacion YUV420 de rpicam-vid en vez de la camara (vacio = camara)
    int camara;             // CamaraVivo: auto = la de la Pi y, si no esta, la USB
    char usb[128];          // dispositivo de la camara USB (vacio = la primera que haya)
} ParamsVivo;

typedef struct {
    Intrinsecos cam; // [camara]
    ParamsCamara camara; // [camara]
    ParamsNmpc nmpc; // [nmpc]
    ParamsComunicacion com; // [comunicacion]
    ParamsEjecutor ejecutor; // [ejecutor]
    ParamsPlanta planta; // [planta]
    ParamsTiempos tiempos; // [tiempos]
    ParamsLazo lazo; // [lazo]
    ParamsLimites lim; // [limites]
    int perfil_ekf;  // [ekf] perfil: PerfilEkf
    ConfigEKF ekf;   // [ekf]: filtro en la pieza; sus defaults dependen de perfil_ekf
    ParamsDashboard dashboard; // [dashboard]
    ParamsVivo vivo;           // [vivo]
} Params;

// Perfil del EKF (informe, tabla de perfiles). Si el INI no lo fija, sale de general.perfil:
// laser -> laser_lento, objeto (avion de juguete) -> avion.
typedef enum { PERFIL_LASER_LENTO = 0, PERFIL_LASER_RAPIDO, PERFIL_LASER_BRUSCO, PERFIL_AVION,
               PERFIL_DESCONOCIDO, PERFIL_N } PerfilEkf;

Params params_por_defecto(void);

// Valores por defecto del EKF para el perfil p->perfil_ekf (informe, tabla de perfiles):
// laser con dos modos de Singer y rho considerado (pared), avion con phi_k y rho estimado por el
// tamano aparente, desconocido con parametros conservadores y rho nominal.
void params_ekf_por_perfil(Params *p);
// Nombre del perfil del EKF (la seccion [ekf_<nombre>]); "?" fuera de rango.
const char *params_nombre_perfil_ekf(int perfil);

// Claves que se pueden cambiar desde el dashboard (POST /ajustes escribe dashboard.ajustes, que
// se aplica sobre torreta.ini al arrancar). Termina en NULL.
extern const char *const PARAMS_CLAVES_AJUSTE[];

// Lee todas las secciones de C ([camara], [lazo], [limites], [ekf], ...). Los
// periodos del filtro y del MPC quedan iguales al del lazo.
void params_leer(Ini *ini, Params *p);

#ifdef __cplusplus
}
#endif

#endif /* PARAMS_H */

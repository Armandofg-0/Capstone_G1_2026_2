// Bucle del modo vivo (ver correr_vivo.hpp). El hilo del lazo toma el cuadro de la fuente, corre la
// cadena del nivel (percepción, EKF y control en sombra, cadena.hpp), llena DatosCuadro y llama a
// Publicador::publicar, que escribe en el triple buffer; el JPEG, los sockets y el resto del
// dashboard viven en el hilo del publicador.
#include "correr_vivo.hpp"
#include "cadena.hpp"
#include "calibracion_camara.hpp"
#include "datos_cuadro.h"
#include "enlace_serie.h"
#include "mensajes.h"
#include "trama.h"
#include "modos.hpp"
#include "publicador.hpp"
#include "reloj_pc.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// src/dashboard/pagina.html embebida por cmake/embeber.cmake (pagina_embebida.c, en el build):
extern "C" {
extern const unsigned char pagina_html[];
extern const size_t pagina_html_n;
}
//--------------------------------------------------------------------------------------------------

namespace { // Funciones y datos solo de uso interno (para que no choquen con otro archivo luego)

// Muestras de us_publicar, ms_percepcion y ms_ctl para el resumen: las últimas N_MUESTRAS (36 min
// a 30 fps), en arreglos reservados al arrancar para no asignar memoria en el hilo del lazo.
const size_t N_MUESTRAS = 1u << 16;

// Escucha de la UART al arrancar (ms) y espera tras una accion de la pagina, para que salga la
// respuesta HTTP antes de terminar el proceso (ms).
const double SONDEO_MS = 300.0;
const int ESPERA_ACCION_MS = 300;

// Buffer circular de muestras: agregar sobrescribe las más antiguas cuando se llena.
struct Muestras {
    std::vector<double> v = std::vector<double>(N_MUESTRAS);
    size_t n = 0; // Muestras agregadas en total (puede superar N_MUESTRAS)
    void agregar(double x) { v[n++ % N_MUESTRAS] = x; }
    std::vector<double> todas() const
    {
        return std::vector<double>(v.begin(), v.begin() + (std::ptrdiff_t)std::min(n, N_MUESTRAS));
    }
};

// flag de Ctrl+C: la activa el handler de la señal y la lee el bucle.
std::atomic<bool> interrumpido{false};

// Se ejecuta cuando el usuario aprieta Ctrl+C (senal SIGINT): solo levanta la bandera y el
// bucle termina ordenadamente en la vuelta siguiente, imprimiendo el resumen. extern "C" le da
// la forma de una funcion de C, que es lo que espera std::signal.
extern "C" void al_interrumpir(int) { interrumpido = true; }
//--------------------------------------------------------------------------------------------------
// Crea el publicador del dashboard, si está habilitado, con el nivel y el candado fijados antes de
// iniciar (el hilo del publicador los pasa al servidor en su primera vuelta). Devuelve false si no
// pudo escuchar (ya lo informó).
bool abrirPublicador(const ParamsDashboard &pd, int nivel, bool bloqueado, const std::string &sistema,
                     std::unique_ptr<Publicador> &pub)
{
    if (!pd.habilitado) return true;
    pub.reset(new Publicador(pd, std::string(reinterpret_cast<const char *>(pagina_html), pagina_html_n)));
    pub->fijarNivel(nivel, bloqueado);
    pub->fijarSistema(sistema);
    if (pub->iniciar()) return true;
    std::printf("dashboard: no se pudo escuchar en %s:%d\n", pd.escucha, pd.puerto);
    pub.reset();
    return false;
}
//--------------------------------------------------------------------------------------------------
// Chequeo de la ESP32 al arrancar: escucha la UART unos ms y resume la telemetria (sin plan la
// ESP32 se queda quieta, asi que es seguro en cualquier nivel). abierto = 0 si el puerto no abre.
struct SondeoEsp32 {
    bool abierto = false;
    long telemetrias = 0;
    int banderas = 0; // OR de las TEL_*
    double q[2] = {NAN, NAN};
};
SondeoEsp32 sondearEsp32(const char *puerto, int baudios, double ms)
{
    SondeoEsp32 r;
    const int fd = serie_abrir(puerto, baudios);
    if (fd < 0) return r;
    r.abierto = true;
    LectorTrama l;
    trama_iniciar(&l);
    uint8_t buf[512];
    for (const double t0 = reloj_pc_ms(); reloj_pc_ms() - t0 < ms;) {
        int n;
        while ((n = serie_leer(fd, buf, sizeof buf)) > 0)
            for (int i = 0; i < n; ++i) {
                MsgTelemetria t;
                const int largo = trama_agregar(&l, buf[i]);
                if (largo != MSG_TELEMETRIA_BYTES || !msg_telemetria_decodificar(l.salida, largo, &t)) continue;
                r.telemetrias++;
                r.banderas |= t.banderas;
                r.q[0] = t.q_hat[0];
                r.q[1] = t.q_hat[1];
            }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    serie_cerrar(fd);
    return r;
}

// Enlace con la ESP32 en los niveles 0 a 3: la Pi no manda planes,
// asi que la ESP32 se queda quieta; se lee su telemetria y su estado de calibracion, y se le envian
// los comandos de calibracion de la pagina. En el nivel 4 la UART es de la cadena.
struct EnlaceCalibracion {
    int fd = -1;
    LectorTrama l;
    double t_tel_ms = -1e12; // llegada de la ultima telemetria
    int banderas = 0;        // TEL_* de las telemetrias de la ultima lectura
    int calib = -1;          // CALIB_* del ultimo estado (-1: no llego ninguno)
    double q[2] = {NAN, NAN}, qmin[2] = {NAN, NAN}, qmax[2] = {NAN, NAN};
    int sentido[2] = {SENTIDO_NINGUNO, SENTIDO_NINGUNO};
    double dq_sentido[2] = {NAN, NAN};

    bool abrir(const char *puerto, int baudios)
    {
        trama_iniciar(&l);
        fd = serie_abrir(puerto, baudios);
        return fd >= 0;
    }
    ~EnlaceCalibracion() { serie_cerrar(fd); }

    // Lee lo que llego desde la vuelta anterior.
    void leer(double ahora_ms)
    {
        uint8_t buf[2048];
        int n, b = 0;
        bool hubo = false;
        while (fd >= 0 && (n = serie_leer(fd, buf, sizeof buf)) > 0)
            for (int i = 0; i < n; ++i) {
                const int largo = trama_agregar(&l, buf[i]);
                MsgTelemetria t;
                MsgEstadoCalib e;
                if (largo == MSG_TELEMETRIA_BYTES && msg_telemetria_decodificar(l.salida, largo, &t)) {
                    t_tel_ms = ahora_ms;
                    q[0] = t.q_hat[0];
                    q[1] = t.q_hat[1];
                    b |= t.banderas;
                    hubo = true;
                } else if (largo == MSG_ESTADO_CALIB_BYTES && msg_estado_calib_decodificar(l.salida, largo, &e)) {
                    calib = e.banderas;
                    for (int j = 0; j < 2; ++j) {
                        qmin[j] = e.q_min[j];
                        qmax[j] = e.q_max[j];
                        sentido[j] = e.sentido[j];
                        dq_sentido[j] = e.dq_sentido[j];
                    }
                }
            }
        if (hubo) banderas = b;
    }

    void enviar(int orden, int eje, float valor)
    {
        const MsgComando c = {(uint8_t)orden, (uint8_t)eje, valor};
        uint8_t m[MSG_COMANDO_BYTES], t[2 * MSG_COMANDO_BYTES + 2];
        msg_comando_codificar(&c, m);
        serie_escribir(fd, t, trama_codificar(m, MSG_COMANDO_BYTES, t));
    }

    // Condiciones para arrancar en el nivel 4: telemetria reciente, sin fallas, calibracion
    // guardada y fuera del modo calibracion.
    const char *motivoNoListo(double ahora_ms) const
    {
        if (ahora_ms - t_tel_ms > 500.0) return "sin telemetria reciente de la ESP32";
        if (banderas & (TEL_DRIVERS_OFF | TEL_FALLA_ENCODER | TEL_FALLA_INA)) return "falla de drivers o sensores";
        if (calib < 0 || !(calib & CALIB_GUARDADA)) return "sin calibracion guardada";
        if (calib & CALIB_CALIBRANDO) return "la ESP32 esta en modo calibracion";
        return nullptr;
    }
};

// Marca que pide arrancar una vez en el nivel 4: junto a ajustes.ini (main la consume).
std::string rutaMarcaNivel4(const char *ajustes)
{
    const std::string a = ajustes;
    const size_t barra = a.find_last_of("/\\");
    return (barra == std::string::npos ? std::string() : a.substr(0, barra + 1)) + "arranque_nivel4";
}

// Texto JSON entre comillas (las claves y valores del INI no traen comillas ni barras, pero se
// escapan igual).
std::string jsonTexto(const std::string &s)
{
    std::string r = "\"";
    for (char ch : s) {
        if (ch == '"' || ch == '\\') r += '\\';
        if ((unsigned char)ch >= 0x20) r += ch;
    }
    return r + "\"";
}

// JSON de GET /sistema: configuracion cargada, claves editables y chequeo de la ESP32.
std::string jsonSistema(const Config &cfg, const SondeoEsp32 &e)
{
    char num[160];
    std::string j = "{\"perfil\":" + jsonTexto(cfg.perfil) +
                    ",\"perfil_ekf\":" + jsonTexto(params_nombre_perfil_ekf(cfg.p.perfil_ekf)) +
                    ",\"fuente\":" + jsonTexto(cfg.fuente);
    std::snprintf(num, sizeof num, ",\"nivel\":%d,\"fps\":%.1f,\"puerto\":", cfg.p.vivo.nivel, cfg.p.vivo.fps);
    j += num + jsonTexto(cfg.p.vivo.puerto) + ",\"ajustes\":{";
    for (size_t i = 0; i < cfg.ajustables.size(); i++)
        j += (i ? "," : "") + jsonTexto(cfg.ajustables[i].first) + ":" + jsonTexto(cfg.ajustables[i].second);
    std::snprintf(num, sizeof num, "},\"esp32\":{\"abierto\":%s,\"telemetrias\":%ld,\"banderas\":%d,\"q\":[",
                  e.abierto ? "true" : "false", e.telemetrias, e.banderas);
    j += num;
    for (int k = 0; k < 2; k++) {
        if (std::isfinite(e.q[k])) std::snprintf(num, sizeof num, "%s%.4f", k ? "," : "", e.q[k]);
        else std::snprintf(num, sizeof num, "%snull", k ? "," : "");
        j += num;
    }
    return j + "]}}";
}
//--------------------------------------------------------------------------------------------------
// Imprime el p50 y el p99 de una serie de muestras, para el resumen final.
void imprimirPercentiles(const char *nombre, const Muestras &m, const char *unidad)
{
    const std::vector<double> v = m.todas();
    std::printf("; %s p50 %.3f, p99 %.3f %s", nombre, percentil(v, 50), percentil(v, 99), unidad);
}

} // namespace
//--------------------------------------------------------------------------------------------------
// Rechaza las combinaciones de [vivo] sin sentido (hoy ninguna: el nivel 4 se valida al abrir
// la UART, en correrVivo).
bool vivoConfigValida(const ParamsVivo &v)
{
    (void)v;
    return true;
}
//--------------------------------------------------------------------------------------------------
// Bucle principal del modo vivo: toma cuadros de la fuente, corre la cadena según el nivel y
// publica al dashboard hasta que se acabe el tiempo, se pulse Ctrl+C o la fuente falle.
int correrVivo(const Config &cfg, std::unique_ptr<Fuente> fuente)
{
    const ParamsVivo &v = cfg.p.vivo;
    const ParamsDashboard &pd = cfg.p.dashboard;
    if (!vivoConfigValida(v)) return 1;
    if (!fuente) return 1; // crearFuente ya dijo por qué

    // Se abren el dashboard (si está habilitado) y la fuente antes de armar la cadena:
    // Chequeo de la ESP32 antes de que el nivel 4 tome la UART.
    const SondeoEsp32 esp = sondearEsp32(v.puerto, (int)cfg.p.tiempos.baudios, SONDEO_MS);
    std::printf("vivo: ESP32 en %s: %s, %ld telemetrias, banderas 0x%02x\n", v.puerto,
                esp.abierto ? "abierto" : "no abre", esp.telemetrias, esp.banderas);

    // Arrancado en el nivel 4, el nivel queda bloqueado: se sale reiniciando el programa.
    const bool bloqueado = v.nivel == 4;
    std::unique_ptr<Publicador> pub;
    if (!abrirPublicador(pd, v.nivel, bloqueado, jsonSistema(cfg, esp), pub)) return 1;
    if (!fuente->abrir()) {
        std::fprintf(stderr, "vivo: no se pudo abrir la fuente %s\n", fuente->nombre());
        return 1;
    }

    // En los niveles 0 a 3 la UART queda para la calibracion.
    EnlaceCalibracion cal;
    if (v.nivel < 4 && !cal.abrir(v.puerto, (int)cfg.p.tiempos.baudios))
        std::fprintf(stderr, "vivo: no se pudo abrir la UART %s para la calibracion\n", v.puerto);

    // Calibracion de la camara desde la pagina (solo en el nivel 0) y su ultimo resultado.
    CalibracionCamara cam(cfg.tablero_cols, cfg.tablero_filas);
    int cam_ultima = -1;
    ResultadoCalibracion cam_res;

    // La cadena corre en los niveles 1 a 3 sobre la fuente.
    std::unique_ptr<Cadena> cadena(new Cadena(cfg, v.nivel, bloqueado));
    if (v.nivel == 4 && !cadena->abrirSerie(v.puerto, (int)cfg.p.tiempos.baudios)) {
        std::fprintf(stderr, "vivo: el nivel 4 no pudo abrir la UART %s a %.0f baudios\n", v.puerto,
                     cfg.p.tiempos.baudios);
        return 1;
    }
    int nivel = cadena->nivel();
    std::unique_ptr<EntradaCtl> in(new EntradaCtl);
    std::unique_ptr<SalidaCtl> out(new SalidaCtl);

    // Acumuladores del resumen final:
    Muestras us, ms_perc, ms_ctl;
    double us_previo = NAN; // Costo de publicar el cuadro anterior (us)
    long long cuadros = 0;
    double t_primero = NAN, t_ultimo = NAN; // Llegada del primer y del último cuadro (ms)
    int ancho = 0, alto = 0, vacias = 0, codigo = 0;

    // Ctrl+C solo levanta el flag; el bucle termina con el cuadro en curso.
    interrumpido = false;
    void (*previo)(int) = std::signal(SIGINT, al_interrumpir);
    const double t0 = reloj_pc_ms();
    while (!interrumpido) {
        if (v.segundos > 0 && reloj_pc_ms() - t0 >= v.segundos * 1e3) break;

        // Se toma el siguiente cuadro; si la fuente falla, se termina con código 1:
        CuadroFuente c;
        if (!fuente->siguiente(c)) {
            std::fprintf(stderr, "vivo: la fuente %s dejo de entregar cuadros\n", fuente->nombre());
            codigo = 1;
            break;
        }

        // Un cuadro vacío no se publica; con VACIAS_MAX seguidos, la fuente se da por caída.
        if (c.img.empty()) {
            if (++vacias < VACIAS_MAX) continue;
            std::fprintf(stderr, "vivo: la fuente %s entrega cuadros vacios\n", fuente->nombre());
            codigo = 1;
            break;
        }
        vacias = 0;

        // Un pedido de la página vale desde este cuadro. Solo 0..3 se aplican en vivo (el servidor
        // ya rechaza el 4; la cadena lo ignora igual).
        int pedido;
        if (pub && pub->nivelPedido(pedido) && !bloqueado && pedido >= 0 && pedido <= 3 && pedido != nivel) {
            cadena->fijarNivel(pedido);
            nivel = cadena->nivel();
            pub->fijarNivel(nivel, false);
        }

        // Calibracion (niveles 0 a 3): se lee la ESP32 y se le pasan los comandos de la pagina.
        const double ahora = reloj_pc_ms();
        cal.leer(ahora);
        int orden, eje;
        float valor;
        if (pub && pub->calibracionPedida(orden, eje, valor)) {
            const bool siempre = orden == CMD_PARAR || orden == CMD_REINICIAR;
            if (cal.fd >= 0) cal.enviar(orden, eje, valor);
            else if (!(siempre && cadena->enviarComando(orden)))
                std::printf("vivo: comando %d ignorado (UART no disponible o nivel 4)\n", orden);
            // Una parada en el nivel 4 ademas deja de planificar: se vuelve al nivel de los ajustes.
            if (orden == CMD_PARAR && nivel == 4 && pub) pub->pedirAccion(ACCION_REINICIAR_PROGRAMA);
        }

        // Reinicio pedido por la página (ajustes guardados o botón): el proceso termina con el
        // código de la acción y systemd lo relanza, o reinicia la Pi (tools/pi/torreta.service).
        int accion = pub ? pub->accionPedida() : ACCION_NINGUNA;
        if (accion == ACCION_NIVEL4) {
            // Se acepta solo con el chequeo en verde; la marca vale para un solo arranque.
            const char *motivo = cal.fd >= 0 ? cal.motivoNoListo(ahora) : "la UART no esta disponible";
            if (motivo) {
                std::printf("vivo: arranque en nivel 4 rechazado: %s\n", motivo);
                accion = ACCION_NINGUNA;
            } else {
                std::FILE *f = std::fopen(rutaMarcaNivel4(pd.ajustes).c_str(), "w");
                if (f) std::fclose(f);
                accion = ACCION_REINICIAR_PROGRAMA;
            }
        }
        if (accion != ACCION_NINGUNA) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ESPERA_ACCION_MS));
            std::printf("vivo: reinicio pedido desde el dashboard (codigo %d)\n", accion);
            codigo = accion;
            break;
        }

        // Calibracion de la camara: solo en el nivel 0, porque calcular tarda segundos y en ese
        // nivel no corre nada que dependa del tiempo.
        const int pedido_cam = pub ? pub->camaraPedida() : CAMARA_NINGUNO;
        if (pedido_cam != CAMARA_NINGUNO && nivel != 0)
            std::printf("vivo: la calibracion de la camara solo se hace en el nivel 0\n");
        else if (pedido_cam == CAMARA_CAPTURAR)
            cam_ultima = cam.capturar(c.img) ? 1 : 0;
        else if (pedido_cam == CAMARA_CALCULAR)
            cam_res = cam.calcular();
        else if (pedido_cam == CAMARA_BORRAR) {
            cam.borrar();
            cam_ultima = -1;
            cam_res = ResultadoCalibracion();
        }

        // Recuadro marcado en la página (perfil general): se siembra en este cuadro.
        int sel[4];
        if (pub && pub->seleccionPedida(sel))
            cadena->seleccionar(sel[0], sel[1], sel[2], sel[3]);

        // Datos mínimos del cuadro para el dashboard:
        DatosCuadro d;
        datos_cuadro_vaciar(&d);
        d.k = c.k;
        d.t_captura_ms = c.t_sof_ms;
        d.nivel = nivel;
        d.bloqueado = bloqueado ? 1 : 0;
        if (cal.fd >= 0) {
            d.esp_ok = ahora - cal.t_tel_ms <= 500.0;
            d.esp_calib = cal.calib;
            d.banderas_esp32 = cal.banderas;
            for (int j = 0; j < 2; ++j) {
                d.esp_q[j] = cal.q[j];
                d.esp_qmin[j] = cal.qmin[j];
                d.esp_qmax[j] = cal.qmax[j];
                d.esp_sentido[j] = cal.sentido[j];
                d.esp_dq[j] = cal.dq_sentido[j];
            }
        }
        d.cam_fotos = cam.fotos();
        d.cam_ultima = cam_ultima;
        d.cam_ok = cam_res.ok;
        if (cam_res.ok) {
            d.cam_rms = cam_res.rms;
            d.cam_K[0] = cam_res.fu;
            d.cam_K[1] = cam_res.fv;
            d.cam_K[2] = cam_res.cu;
            d.cam_K[3] = cam_res.cv;
            d.cam_k[0] = cam_res.k1;
            d.cam_k[1] = cam_res.k2;
        }
        std::snprintf(d.fuente, sizeof d.fuente, "%s", fuente->nombre());
        d.us_publicar = us_previo;

        // Cadena según el nivel: percepción (1+), y estimación y control en sombra (2+).
        if (nivel >= 1) {
            // Niveles 1 a 3: ejes fijos en q0. Nivel 4: la pose de la última telemetría de la ESP32.
            double t = reloj_pc_ms();
            const MedicionCamara m = cadena->percibir(c, nivel == 4 ? cadena->qMedida() : v.q0, d);
            d.ms_percepcion = reloj_pc_ms() - t;
            ms_perc.agregar(d.ms_percepcion);
            if (nivel >= 2) {
                cadena->armarEntrada(m, reloj_pc_ms(), *in);
                t = reloj_pc_ms();
                cadena->controlar(*in, *out, d);
                d.ms_ctl = reloj_pc_ms() - t;
                ms_ctl.agregar(d.ms_ctl);
                cadena->enviar(*out, reloj_pc_ms());
            }
        }

        // Publicación al dashboard (si está habilitado); se mide su costo para el próximo cuadro:
        if (pub) {
            const double t = reloj_pc_ms();
            pub->publicar(c.img, d);
            us_previo = (reloj_pc_ms() - t) * 1e3;
            us.agregar(us_previo);
        }

        // Registro del primer cuadro (resolución) y del último (para los fps):
        if (cuadros == 0) {
            t_primero = c.t_llegada_ms;
            ancho = c.img.cols;
            alto = c.img.rows;
        }
        t_ultimo = c.t_llegada_ms;
        cuadros++;
    }
    std::signal(SIGINT, previo == SIG_ERR ? SIG_DFL : previo);

    // Resumen final. Los fps se miden entre el primer y el último cuadro: la duración que se
    // informa es esa misma, para que cuadros y fps cuadren. La apertura de la cámara
    // (calentamiento hasta el primer cuadro) se informa aparte.
    const double fps = cuadros > 1 ? (cuadros - 1) * 1e3 / (t_ultimo - t_primero) : 0.0;
    const double s_cuadros = cuadros > 1 ? (t_ultimo - t_primero) * 1e-3 : 0.0;
    const double s_arranque = cuadros > 0 ? (t_primero - t0) * 1e-3 : 0.0;
    std::printf("vivo: fuente %s %dx%d, %lld cuadros en %.2f s (%.1f fps), arranque %.2f s", fuente->nombre(),
                ancho, alto, cuadros, s_cuadros, fps, s_arranque);
    if (us.n > 0) {
        const std::vector<double> m = us.todas();
        std::printf("; us_publicar p50 %.1f us, p99 %.1f us\n", percentil(m, 50), percentil(m, 99));
    } else {
        std::printf("; sin publicar (dashboard apagado)\n");
    }
    if (ms_perc.n > 0) {
        const std::vector<double> p = ms_perc.todas();
        std::printf("vivo: ms_percepcion p50 %.3f, p99 %.3f ms (%zu cuadros)", percentil(p, 50), percentil(p, 99),
                    p.size());
        if (ms_ctl.n > 0) {
            const std::vector<double> m = ms_ctl.todas();
            std::printf("; ms_ctl p50 %.3f, p99 %.3f ms (%zu cuadros)", percentil(m, 50), percentil(m, 99), m.size());
        }
        std::printf("\n");
    }
    if (v.nivel == 4) {
        const Cadena::EstadisticaEnlace &e = cadena->enlace();
        std::printf("vivo: enlace %ld telemetrias, %ld con plan en ejecucion, ultima secuencia %d\n", e.telemetrias,
                    e.con_plan, e.ultima_secuencia);
    }
    return codigo;
}

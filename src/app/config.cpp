// Lectura de la configuracion (ver config.hpp).
//
// El proceso tiene dos etapas (ver src/params/ini.h). Primero se juntan todos los valores
// como texto: los del archivo INI y luego los de la linea de comandos. Despues se pide cada
// clave conocida: la parte C con params_leer() y la de la aplicacion y la percepcion aca.
// Al final, cualquier clave que nadie pidio se informa como desconocida.

#include "config.hpp"

#include <cstdio>
#include <fstream>
#include <memory>

// Carpeta raiz del proyecto. Normalmente la define el sistema de compilacion (ver
// CMakeLists.txt); si no, se usa el directorio actual.
#ifndef RUTA_PROYECTO
#define RUTA_PROYECTO "."
#endif

namespace {

// true si el archivo existe (se intenta abrirlo para lectura).
bool existe(const std::string &ruta)
{
    return std::ifstream(ruta).good();
}
//---------------------------------------------------------------------------
// Adaptadores de tipos de C++ a las funciones del lector en C.
void leer(Ini *ini, const char *clave, std::string &x)
{
    if (const char *t = ini_texto(ini, clave))
        x = t;
}
void leer(Ini *ini, const char *clave, bool &x)
{
    int v = x;
    ini_leer_bool(ini, clave, &v);
    x = v != 0;
}
void leer(Ini *ini, const char *clave, float &x) { ini_leer_float(ini, clave, &x); }
void leer(Ini *ini, const char *clave, int &x) { ini_leer_int(ini, clave, &x); }
//---------------------------------------------------------------------------
// Texto que debe ser una de las opciones validas.
void opcion(Ini *ini, const char *clave, std::string &x, std::initializer_list<const char *> validas)
{
    std::vector<const char *> v(validas);
    int i = -1;
    if (ini_leer_opcion(ini, clave, v.data(), (int)v.size(), &i) && i >= 0)
        x = v[i];
}

} // namespace

// Busca un recurso del proyecto. Primero se prueba la ruta tal cual (relativa al
// directorio desde donde se ejecuta el programa); si no existe, se prueba relativa a la
// raiz del proyecto.
std::string rutaProyecto(const std::string &ruta)
{
    if (ruta.empty() || existe(ruta))
        return ruta;
    const std::string r = std::string(RUTA_PROYECTO) + "/" + ruta;
    return existe(r) ? r : ruta;
}
//---------------------------------------------------------------------------
// Carga toda la configuracion. Los nombres de las claves son los mismos que aparecen (con
// su explicacion) en config/torreta.ini.
bool cargarConfig(const std::string &ruta, const std::vector<std::string> &asignaciones, Config &c)
{
    // Ini es grande (cientos de entradas de texto): va en el heap.
    std::unique_ptr<Ini> a(new Ini);
    Ini *ini = a.get();
    ini_iniciar(ini);
    if (!ruta.empty() && !ini_cargar_archivo(ini, ruta.c_str()))
        return false;
    for (const std::string &s : asignaciones)
        if (!ini_asignar(ini, s.c_str(), "linea de comandos"))
            return false;

    for (int i = 0; PARAMS_CLAVES_AJUSTE[i]; i++) {
        const char *v = ini_texto(ini, PARAMS_CLAVES_AJUSTE[i]);
        c.ajustables.emplace_back(PARAMS_CLAVES_AJUSTE[i], v ? v : "");
    }

    // Con la camara USB, [usb] pisa lo que cambia con ella: sus intrinsecos y lo que se cuenta en
    // cuadros (va a 15 fps). Despues de anotar los ajustables, que el dashboard guarda con su
    // propio nombre.
    if (c.fuente == "usb")
        ini_superponer_seccion(ini, "usb");
    else
        ini_ignorar_seccion(ini, "usb");

    // [general]
    opcion(ini, "general.modo", c.modo, {"vivo"});
    opcion(ini, "general.perfil", c.perfil, {"laser", "objeto", "general"});
    leer(ini, "general.csv", c.csv);

    // Parte C. El perfil del EKF sigue al de deteccion, salvo que el INI fije ekf.perfil.
    c.p.perfil_ekf = c.perfil == "objeto"    ? PERFIL_AVION
                     : c.perfil == "general" ? PERFIL_DESCONOCIDO
                                             : PERFIL_LASER_LENTO;
    params_leer(ini, &c.p);

    // [laser]
    ConfigLaser &l = c.laser;
    {
        static const char *const canales[] = {"gris", "rojo", "verde", "azul", "croma_rojo", "croma_verde", "croma_azul"};
        int canal = (int)l.spot.canal;
        ini_leer_opcion(ini, "laser.canal", canales, 7, &canal);
        l.spot.canal = (SpotTracker::Canal)canal;
    }
    leer(ini, "laser.oscuro", l.spot.oscuro);
    leer(ini, "laser.umbral_adquisicion", l.spot.umbral_adquisicion);
    leer(ini, "laser.umbral_seguimiento", l.spot.umbral_seguimiento);
    leer(ini, "laser.radio_blob_max", l.spot.radio_blob_max);
    leer(ini, "laser.decimacion", l.spot.decimacion);
    leer(ini, "laser.confirmaciones", l.spot.confirmaciones);
    leer(ini, "laser.ambiguedad", l.spot.ambiguedad);
    leer(ini, "laser.contraste_min", l.spot.contraste_min);
    leer(ini, "laser.max_fallos", l.spot.max_fallos);
    leer(ini, "laser.sigma_px", l.sigma_px);
    leer(ini, "laser.snr_ref", l.snr_ref);
    leer(ini, "laser.sigma_max_factor", l.sigma_max_factor);
    leer(ini, "laser.camara", l.camara);

    // [objeto]
    ConfigObjeto &o = c.objeto;
    leer(ini, "objeto.prob_min", o.prob_min);
    leer(ini, "objeto.sigma_px", o.sigma_px);
    leer(ini, "objeto.sigma_max_factor", o.sigma_max_factor);
    leer(ini, "objeto.max_perdidos", o.max_perdidos);
    leer(ini, "objeto.margen_borde", o.margen_borde);
    leer(ini, "objeto.kcf_escala_min", o.kcf.min_scale);
    leer(ini, "objeto.kcf_escala_max", o.kcf.max_scale);
    leer(ini, "objeto.periodo_verificacion", o.periodo_verificacion);
    leer(ini, "objeto.max_verif_fallidas", o.max_verif_fallidas);
    leer(ini, "objeto.iou_ok", o.iou_ok);
    leer(ini, "objeto.centrar_con_pista", o.centrar_con_pista);
    leer(ini, "objeto.kcf_psr_umbral", o.kcf.psr_threshold);
    leer(ini, "objeto.kcf_psr_caida", o.kcf.psr_drop_ratio);
    leer(ini, "objeto.kcf_plantilla", o.kcf.template_size);
    leer(ini, "objeto.kcf_padding", o.kcf.padding);
    leer(ini, "objeto.kcf_aprendizaje", o.kcf.interp_factor);
    leer(ini, "objeto.umbral_rojo", c.rojo.umbral);
    leer(ini, "objeto.area_min", c.rojo.area_min);
    leer(ini, "objeto.cierre", c.rojo.cierre);
    leer(ini, "objeto.peso_movimiento", c.rojo.peso_movimiento);
    leer(ini, "objeto.camara", c.objeto_camara);

    leer(ini, "camara.tablero_cols", c.tablero_cols);
    leer(ini, "camara.tablero_filas", c.tablero_filas);

    // [seleccion]: perfil general
    ConfigGeneral &g = c.general;
    leer(ini, "seleccion.sigma_px", g.sigma_px);
    leer(ini, "seleccion.sigma_max_factor", g.sigma_max_factor);
    leer(ini, "seleccion.max_perdidos", g.max_perdidos);
    leer(ini, "seleccion.cuadros_reintento", g.cuadros_reintento);
    leer(ini, "seleccion.confirmaciones_reintento", g.confirmaciones_reintento);
    leer(ini, "seleccion.margen_borde", g.margen_borde);
    leer(ini, "seleccion.kcf_psr_umbral", g.kcf.psr_threshold);
    leer(ini, "seleccion.kcf_psr_caida", g.kcf.psr_drop_ratio);
    leer(ini, "seleccion.kcf_plantilla", g.kcf.template_size);
    leer(ini, "seleccion.kcf_padding", g.kcf.padding);
    leer(ini, "seleccion.kcf_aprendizaje", g.kcf.interp_factor);
    leer(ini, "seleccion.kcf_escala_min", g.kcf.min_scale);
    leer(ini, "seleccion.kcf_escala_max", g.kcf.max_scale);
    leer(ini, "seleccion.camara", c.general_camara);

    // [barrida], comun a los perfiles laser y objeto
    ConfigBarrida &b = c.laser.barrida;
    ini_leer_bool(ini, "barrida.habilitada", &b.habilitada);
    leer(ini, "barrida.periodo_cuadros", b.periodo_cuadros);
    leer(ini, "barrida.n_confirmaciones", b.n_confirmaciones);
    leer(ini, "barrida.umbral_movimiento", b.umbral_movimiento);
    leer(ini, "barrida.radio_exclusion", b.radio_exclusion);
    leer(ini, "barrida.radio_asociacion", b.radio_asociacion);
    leer(ini, "barrida.radio_primer_salto", b.radio_primer_salto);
    c.objeto.barrida = b;

    // Por ultimo, se informan las claves desconocidas (y se devuelve si hubo errores).
    return ini_terminar(ini) != 0;
}

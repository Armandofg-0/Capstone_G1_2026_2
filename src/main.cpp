// torreta: punto de entrada del sistema en la Raspberry Pi.
//
// Forma de uso:
//
//     torreta [--config archivo.ini] [seccion.clave=valor ...]
//
// Sin --config se usa el archivo config/torreta.ini del proyecto. Cada seccion.clave=valor
// que se escriba pisa lo que dice el archivo, y si no se indica la seccion, se entiende
// que es [general]. El unico modo es "vivo": percepcion (perfil laser u objeto, el avion de
// juguete), EKF y NMPC sobre la camara, publicando al dashboard.
//
// Entre el archivo y la linea de comandos se aplica config/ajustes.ini (lineas
// seccion.clave=valor que escribe el dashboard). Si con ellos la configuracion no carga, el
// archivo se renombra a ajustes.ini.rechazado y se arranca sin ellos. La marca
// config/arranque_nivel4 (la deja el dashboard) pide arrancar una vez en el nivel 4. Si la
// camara de la Pi no esta, se usa una webcam USB con los parametros de [usb].

#include "config.hpp"
#include "fuente.hpp"
#include "modos.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Imprime la ayuda del programa.
void uso()
{
    std::printf("uso: torreta [--config archivo.ini] [seccion.clave=valor ...]\n"
                "  sin --config se lee %s\n"
                "  perfiles (general.perfil): laser | objeto (avion de juguete)\n"
                "  ejemplo: torreta perfil=laser vivo.nivel=3\n"
                "  todas las claves, con su explicacion, estan en config/torreta.ini\n",
                rutaProyecto("config/torreta.ini").c_str());
}

// Lineas no vacias de un archivo de ajustes (vacio si no existe).
std::vector<std::string> leerAjustes(const std::string &ruta)
{
    std::vector<std::string> v;
    std::ifstream f(ruta);
    for (std::string l; std::getline(f, l);) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        if (!l.empty() && l[0] != ';' && l[0] != '#') v.push_back(l);
    }
    return v;
}

} // namespace

// argc es la cantidad de argumentos de la linea de comandos y argv son esos argumentos
// como textos (argv[0] es el nombre del propio programa, por eso se empieza en 1).
int main(int argc, char **argv)
{
    std::string ruta = rutaProyecto("config/torreta.ini");
    std::vector<std::string> asignaciones;
    for (int i = 1; i < argc; i++)
    {
        // strcmp devuelve 0 cuando los textos son iguales, por eso se usa !strcmp.
        // "--config" consume tambien el argumento siguiente (la ruta del archivo).
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            ruta = argv[++i];
        else if (!std::strcmp(argv[i], "--ayuda") || !std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help"))
        {
            uso();
            return 0;
        }
        // Todo lo que tenga un "=" se trata como una asignacion seccion.clave=valor.
        else if (std::strchr(argv[i], '='))
            asignaciones.push_back(argv[i]);
        else
        {
            std::fprintf(stderr, "argumento desconocido: %s\n\n", argv[i]);
            uso();
            return 1;
        }
    }

    const std::string ruta_ajustes = rutaProyecto("config/ajustes.ini");
    // Arranque en el nivel 4 pedido desde el dashboard: vale una sola vez (la marca se borra), asi
    // que tras un corte de luz se vuelve al nivel de los ajustes.
    const std::string marca_nivel4 = rutaProyecto("config/arranque_nivel4");
    if (std::ifstream(marca_nivel4).good()) {
        std::remove(marca_nivel4.c_str());
        asignaciones.push_back("vivo.nivel=4");
        std::printf("torreta: arranque en nivel 4 pedido desde el dashboard\n");
    }
    std::vector<std::string> con_ajustes = leerAjustes(ruta_ajustes);
    con_ajustes.insert(con_ajustes.end(), asignaciones.begin(), asignaciones.end());
    Config cfg;
    if (!cargarConfig(ruta, con_ajustes, cfg)) {
        if (con_ajustes.size() == asignaciones.size())
            return 1;
        std::fprintf(stderr, "torreta: %s no carga; se aparta y se arranca sin ajustes\n", ruta_ajustes.c_str());
        std::rename(ruta_ajustes.c_str(), (ruta_ajustes + ".rechazado").c_str());
        cfg = Config();
        con_ajustes = asignaciones;
        if (!cargarConfig(ruta, con_ajustes, cfg))
            return 1;
    }

    // Camara: la de la Pi o, si no esta, la USB (vivo.camara). Con la USB se vuelve a cargar la
    // configuracion aplicando [usb].
    std::string usb;
    if (elegirCamara(cfg.p.vivo, usb) == "usb") {
        con_ajustes.push_back("vivo.usb=" + usb);
        cfg = Config();
        cfg.fuente = "usb";
        if (!cargarConfig(ruta, con_ajustes, cfg))
            return 1;
    }
    std::snprintf(cfg.p.dashboard.ajustes, sizeof cfg.p.dashboard.ajustes, "%s", ruta_ajustes.c_str());
    std::printf("torreta: modo %s, perfil %s (configuracion: %s)\n", cfg.modo.c_str(), cfg.perfil.c_str(),
                ruta.c_str());

    return correrVivo(cfg);
}

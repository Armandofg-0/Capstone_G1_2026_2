// Modo "vivo": despacha al bucle de src/vivo con la camara elegida al arrancar (picam o USB).
#include "correr_vivo.hpp"
#include "modos.hpp"

// Punto de entrada del modo vivo: valida la configuracion, crea la camara y corre el bucle.
int correrVivo(const Config &cfg)
{
    if (!vivoConfigValida(cfg.p.vivo)) return 1;
    return correrVivo(cfg, crearFuente(cfg.fuente, cfg.p.vivo, cfg.perfil == "laser" ? cfg.laser.camara : cfg.perfil == "general" ? cfg.general_camara : cfg.objeto_camara));
}

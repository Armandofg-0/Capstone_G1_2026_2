# Script previo de PlatformIO: genera src/config.h desde config/torreta.ini, para que pines y
# parametros del ejecutor tengan un solo origen (el mismo archivo que lee la Pi).
import configparser
import os

Import("env")  # noqa: F821 (lo define PlatformIO)

raiz = env.subst("$PROJECT_DIR")  # noqa: F821
ini = configparser.ConfigParser(inline_comment_prefixes=(";", "#"))
ini.optionxform = str
ini.read(os.path.join(raiz, "..", "config", "torreta.ini"), encoding="utf-8")


# Valor de seccion.clave tal como esta escrito.
def v(seccion, clave):
    return ini.get(seccion, clave).strip()


# Par "pan, tilt" como inicializador de C.
def par(clave):
    return "{" + ", ".join(x.strip() for x in v("esp32", clave).split(",")) + "}"


lineas = [
    "// GENERADO por gen_config.py desde config/torreta.ini. No editar: cambiar el INI.",
    "#pragma once",
    "",
    "// [esp32]: pines por eje (0 = pan, 1 = tilt), cada uno con su bus I2C",
]
for c in ["sda", "scl", "en", "m0", "m1", "step", "dir"]:
    lineas.append(f"static const int PIN_{c.upper()}[2] = {par(c)};")
lineas += [
    f"#define PIN_UART_RX {v('esp32', 'uart_rx')}",
    f"#define PIN_UART_TX {v('esp32', 'uart_tx')}",
    f"static const int SIGNO_DIR[2] = {par('signo_dir')};",
    f"static const int SIGNO_ENC[2] = {par('signo_enc')};",
    f"#define PASOS_VUELTA {float(v('esp32', 'pasos_vuelta'))}",
    f"#define R_SHUNT {float(v('esp32', 'r_shunt'))}",
    f"#define I_MAX {float(v('esp32', 'i_max'))}",
    f"#define N_SOBRECORRIENTE {int(v('esp32', 'n_sobrecorriente'))}",
    "",
    "// [tiempos], [lazo], [planta], [limites] y [ejecutor]: los mismos que usa la Pi en su espejo",
    f"#define BAUDIOS {int(float(v('tiempos', 'baudios')))}",
    f"#define F_TELEMETRIA {float(v('tiempos', 'f_telemetria'))}",
    f"#define TS (1.0 / {float(v('lazo', 'fps'))})",
    f"#define DT_EJECUTOR {float(v('planta', 'dt_ejecutor'))}",
    f"static const int MICROPASOS[2] = {{{int(v('planta', 'micropasos_pan'))}, {int(v('planta', 'micropasos_tilt'))}}};",
    f"static const double RELACION[2] = {{{float(v('planta', 'relacion_pan'))}, {float(v('planta', 'relacion_tilt'))}}};",
    f"#define Q_MIN_PAN ({float(v('limites', 'pan_min'))})",
    f"#define Q_MAX_PAN ({float(v('limites', 'pan_max'))})",
    f"#define Q_MIN_TILT ({float(v('limites', 'tilt_min'))})",
    f"#define Q_MAX_TILT ({float(v('limites', 'tilt_max'))})",
    f"#define TILT_ACTIVO {int(ini.getboolean('planta', 'tilt_activo'))}",
    f"#define TILT_FIJO ({float(v('planta', 'tilt_fijo'))})",
]
for c in ["a_lim", "a_frenado", "margen_limite", "qd_max", "kp", "tau_correccion", "alfa", "beta"]:
    lineas.append(f"#define {c.upper()} {float(v('ejecutor', c))}")

with open(os.path.join(raiz, "src", "config.h"), "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(lineas) + "\n")

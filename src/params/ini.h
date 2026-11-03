// Lector de configuracion INI, en C (lo usan la aplicacion C++ y los ejecutables C).
//
// Formato: lineas "[seccion]", lineas "clave = valor" y comentarios que empiezan con ';' o
// '#'. Lo que aparece antes de la primera seccion es de [general]. Un ';' o '#' dentro de
// un valor solo inicia un comentario si va al principio o despues de un espacio; un valor
// entre comillas se toma literal.
//
// Se usa en dos etapas. Primero se juntan los textos: el archivo y luego las asignaciones
// "seccion.clave=valor" de la linea de comandos, que pisan a las del archivo (sin seccion
// se entiende [general]). Despues, cada modulo pide sus claves con ini_leer_*: si la clave
// no se escribio, el destino conserva su valor por defecto; si se escribio con un valor
// invalido, se informa por stderr y el destino no cambia. ini_terminar() informa las
// claves que nadie pidio (errores de tipeo) y dice si hubo algun error.
#ifndef INI_H
#define INI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INI_MAX_ENTRADAS 512

typedef struct {
    char clave[64];   // "seccion.clave"
    char texto[256];  // valor sin comentario ni comillas
    char origen[160]; // "archivo:linea" o "linea de comandos", para los mensajes
    int usada;
} IniEntrada;

typedef struct {
    int n;
    int errores;
    IniEntrada e[INI_MAX_ENTRADAS];
} Ini;

void ini_iniciar(Ini *ini);
// Devuelve 0 si el archivo no se puede leer o tiene un error de formato.
int ini_cargar_archivo(Ini *ini, const char *ruta);
// "seccion.clave=valor". Devuelve 0 si no tiene '='.
int ini_asignar(Ini *ini, const char *asignacion, const char *origen);

// Texto de la clave (marcandola como usada), o NULL si no se escribio.
const char *ini_texto(Ini *ini, const char *clave);

void ini_leer_double(Ini *ini, const char *clave, double *x);
void ini_leer_float(Ini *ini, const char *clave, float *x);
void ini_leer_int(Ini *ini, const char *clave, int *x);
void ini_leer_u64(Ini *ini, const char *clave, uint64_t *x);
// Acepta true/si/1 y false/no/0, sin distinguir mayusculas.
void ini_leer_bool(Ini *ini, const char *clave, int *x);
// Matriz 2x2 diagonal: "w" (ambos ejes) o "w_pan, w_tilt".
void ini_leer_diagonal(Ini *ini, const char *clave, double W[4]);
void ini_leer_texto(Ini *ini, const char *clave, char *dst, size_t n);
// Indice del valor en la lista de opciones. Si la clave no se escribio, *idx no cambia y
// devuelve 1; si el valor no esta en la lista, informa las opciones validas y devuelve 0.
int ini_leer_opcion(Ini *ini, const char *clave, const char *const opciones[], int n, int *idx);

// Marca como leidas todas las claves de [seccion] (las de otro programa).
void ini_ignorar_seccion(Ini *ini, const char *seccion);

// Cada clave de [seccion] se escribe como "seccion.clave = valor" y se aplica como una
// asignacion que pisa o agrega esa clave; las de [seccion] quedan leidas. Se llama antes de leer
// las demas (p. ej. [usb]: lo que cambia con la camara USB).
void ini_superponer_seccion(Ini *ini, const char *seccion);

// Informa las claves que nadie leyo. Devuelve 1 si no hubo ningun error.
int ini_terminar(Ini *ini);

#ifdef __cplusplus
}
#endif

#endif /* INI_H */

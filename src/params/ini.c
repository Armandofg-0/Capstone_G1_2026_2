// Lector INI (ver ini.h).
//
// Un archivo INI es texto con secciones [nombre] y lineas "clave = valor" (config/torreta.ini).
// Este lector guarda todas las claves como texto ("seccion.clave" -> "valor") y despues cada
// modulo pide las suyas con el tipo que necesita (numero, entero, booleano, texto).
//
// Sin memoria dinamica: las entradas viven en un arreglo fijo dentro de Ini. Los errores no
// cortan la lectura: se informan por stderr, se cuentan y se sigue, para que el usuario vea
// todos los problemas de una vez.
#include "ini.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Deja el lector vacio, sin claves ni errores.
void ini_iniciar(Ini *ini)
{
    ini->n = 0;
    ini->errores = 0;
}
//---------------------------------------------------------------------------
// Quita los espacios del comienzo y del final, en el lugar.
static char *recortar(char *s)
{
    while (isspace((unsigned char)*s))
        s++;
    char *f = s + strlen(s);
    while (f > s && isspace((unsigned char)f[-1]))
        *--f = '\0';
    return s;
}
//---------------------------------------------------------------------------
// Copia un texto truncandolo a n - 1 caracteres y siempre termina en '\0'.
static void copiar(char *dst, size_t n, const char *src)
{
    snprintf(dst, n, "%s", src);
}
//---------------------------------------------------------------------------
// Guarda (o pisa) el texto de una clave.
static void fijar(Ini *ini, const char *clave, const char *texto, const char *origen)
{
    int i = 0;
    while (i < ini->n && strcmp(ini->e[i].clave, clave))
        i++;
    if (i == ini->n) {
        if (ini->n == INI_MAX_ENTRADAS) {
            fprintf(stderr, "%s: demasiadas claves (maximo %d)\n", origen, INI_MAX_ENTRADAS);
            ini->errores++;
            return;
        }
        ini->n++;
        copiar(ini->e[i].clave, sizeof(ini->e[i].clave), clave);
    }
    copiar(ini->e[i].texto, sizeof(ini->e[i].texto), texto);
    copiar(ini->e[i].origen, sizeof(ini->e[i].origen), origen);
    ini->e[i].usada = 0;
}
//---------------------------------------------------------------------------
// Valor a la derecha del '=': sin comentario al final y sin comillas. Modifica s.
static char *valor(char *s)
{
    s = recortar(s);
    if (*s == '"') {
        char *fin = strchr(s + 1, '"');
        if (fin)
            *fin = '\0';
        return s + 1;
    }
    for (char *p = s; *p; ++p)
        if ((*p == ';' || *p == '#') && (p == s || isspace((unsigned char)p[-1]))) {
            *p = '\0';
            break;
        }
    return recortar(s);
}
//---------------------------------------------------------------------------
// Lee el archivo linea por linea: recuerda la seccion vigente y guarda cada clave con el
// nombre "seccion.clave". Devuelve 0 si el archivo no se pudo abrir.
int ini_cargar_archivo(Ini *ini, const char *ruta)
{
    FILE *f = fopen(ruta, "r");
    if (!f) {
        fprintf(stderr, "no se pudo leer la configuracion '%s'\n", ruta);
        ini->errores++;
        return 0;
    }
    char linea[512], seccion[64] = "general", clave[160], origen[160];
    for (int n = 1; fgets(linea, sizeof(linea), f); n++) {
        char *l = recortar(linea);
        if (!*l || *l == ';' || *l == '#')
            continue;
        snprintf(origen, sizeof(origen), "%s:%d", ruta, n);
        if (*l == '[') {
            char *fin = strchr(l, ']');
            if (!fin) {
                fprintf(stderr, "%s: falta ']'\n", origen);
                ini->errores++;
                fclose(f);
                return 0;
            }
            *fin = '\0';
            copiar(seccion, sizeof(seccion), recortar(l + 1));
            continue;
        }
        char *igual = strchr(l, '=');
        if (!igual) {
            fprintf(stderr, "%s: se esperaba clave = valor\n", origen);
            ini->errores++;
            fclose(f);
            return 0;
        }
        *igual = '\0';
        snprintf(clave, sizeof(clave), "%s.%s", seccion, recortar(l));
        fijar(ini, clave, valor(igual + 1), origen);
    }
    fclose(f);
    return 1;
}
//---------------------------------------------------------------------------
// Una asignacion "seccion.clave=valor" escrita en la linea de comandos: pisa la del archivo.
int ini_asignar(Ini *ini, const char *asignacion, const char *origen)
{
    char buf[512], clave[160];
    copiar(buf, sizeof(buf), asignacion);
    char *igual = strchr(buf, '=');
    if (!igual || igual == buf) {
        fprintf(stderr, "%s: se esperaba seccion.clave=valor en '%s'\n", origen, asignacion);
        ini->errores++;
        return 0;
    }
    *igual = '\0';
    const char *c = recortar(buf);
    if (strchr(c, '.'))
        copiar(clave, sizeof(clave), c);
    else
        snprintf(clave, sizeof(clave), "general.%s", c);
    fijar(ini, clave, recortar(igual + 1), origen);
    return 1;
}
//---------------------------------------------------------------------------
// Busca una clave y la marca como usada (para avisar despues de las que nadie pidio).
static IniEntrada *tomar(Ini *ini, const char *clave)
{
    for (int i = 0; i < ini->n; ++i)
        if (!strcmp(ini->e[i].clave, clave)) {
            ini->e[i].usada = 1;
            return &ini->e[i];
        }
    return NULL;
}
//---------------------------------------------------------------------------
// Informa y cuenta un valor que no tiene el tipo esperado.
static void invalido(Ini *ini, const char *clave, const IniEntrada *e, const char *esperado)
{
    fprintf(stderr, "%s: %s = '%s': se esperaba %s\n", e->origen, clave, e->texto, esperado);
    ini->errores++;
}
//---------------------------------------------------------------------------
// Texto crudo de una clave, o NULL si no existe.
const char *ini_texto(Ini *ini, const char *clave)
{
    const IniEntrada *e = tomar(ini, clave);
    return e ? e->texto : NULL;
}
//---------------------------------------------------------------------------
// Numero estricto: despues del numero solo pueden quedar espacios. El programa no cambia
// el locale, asi que el separador decimal es siempre el punto.
static int numero(const char *s, double *x)
{
    char *fin;
    errno = 0;
    const double v = strtod(s, &fin);
    if (fin == s || errno == ERANGE)
        return 0;
    while (isspace((unsigned char)*fin))
        fin++;
    if (*fin)
        return 0;
    *x = v;
    return 1;
}
//---------------------------------------------------------------------------
// Entero estricto: como numero(), pero sin decimales.
static int entero(const char *s, long long *x)
{
    char *fin;
    errno = 0;
    const long long v = strtoll(s, &fin, 10);
    if (fin == s || errno == ERANGE)
        return 0;
    while (isspace((unsigned char)*fin))
        fin++;
    if (*fin)
        return 0;
    *x = v;
    return 1;
}
//---------------------------------------------------------------------------
// Las funciones ini_leer_* buscan una clave y, si existe y es valida, escriben el valor en x.
// Si no existe, x conserva su valor por defecto: por eso cada modulo llena primero sus
// defectos y despues lee el INI encima.
void ini_leer_double(Ini *ini, const char *clave, double *x)
{
    const IniEntrada *e = tomar(ini, clave);
    if (e && !numero(e->texto, x))
        invalido(ini, clave, e, "un numero");
}
//---------------------------------------------------------------------------
void ini_leer_float(Ini *ini, const char *clave, float *x)
{
    double v = *x;
    ini_leer_double(ini, clave, &v);
    *x = (float)v;
}
//---------------------------------------------------------------------------
void ini_leer_int(Ini *ini, const char *clave, int *x)
{
    const IniEntrada *e = tomar(ini, clave);
    long long v;
    if (!e)
        return;
    if (entero(e->texto, &v))
        *x = (int)v;
    else
        invalido(ini, clave, e, "un entero");
}
//---------------------------------------------------------------------------
void ini_leer_u64(Ini *ini, const char *clave, uint64_t *x)
{
    const IniEntrada *e = tomar(ini, clave);
    long long v;
    if (!e)
        return;
    if (entero(e->texto, &v) && v >= 0)
        *x = (uint64_t)v;
    else
        invalido(ini, clave, e, "un entero no negativo");
}
//---------------------------------------------------------------------------
void ini_leer_bool(Ini *ini, const char *clave, int *x)
{
    const IniEntrada *e = tomar(ini, clave);
    if (!e)
        return;
    char t[16];
    size_t i = 0;
    for (; e->texto[i] && i < sizeof(t) - 1; ++i)
        t[i] = (char)tolower((unsigned char)e->texto[i]);
    t[i] = '\0';
    if (!strcmp(t, "true") || !strcmp(t, "si") || !strcmp(t, "1"))
        *x = 1;
    else if (!strcmp(t, "false") || !strcmp(t, "no") || !strcmp(t, "0"))
        *x = 0;
    else
        invalido(ini, clave, e, "true o false");
}
//---------------------------------------------------------------------------
// Una matriz diagonal 2x2 escrita como "a, b" (los dos elementos de la diagonal).
void ini_leer_diagonal(Ini *ini, const char *clave, double W[4])
{
    const IniEntrada *e = tomar(ini, clave);
    if (!e)
        return;
    char a[256], b[256];
    const char *coma = strchr(e->texto, ',');
    copiar(a, sizeof(a), e->texto);
    if (coma) {
        a[coma - e->texto] = '\0';
        copiar(b, sizeof(b), coma + 1);
    } else
        copiar(b, sizeof(b), e->texto);
    double wa, wb;
    if (numero(a, &wa) && numero(b, &wb)) {
        W[0] = wa;
        W[1] = W[2] = 0.0;
        W[3] = wb;
    } else
        invalido(ini, clave, e, "un numero o dos separados por coma");
}
//---------------------------------------------------------------------------
// Texto de una clave, copiado a dst (truncado a n - 1 caracteres).
void ini_leer_texto(Ini *ini, const char *clave, char *dst, size_t n)
{
    const IniEntrada *e = tomar(ini, clave);
    if (e)
        copiar(dst, n, e->texto);
}
//---------------------------------------------------------------------------
// Una clave que debe ser una de varias palabras (por ejemplo "laser" u "objeto"): devuelve en
// idx su posicion en la lista de opciones.
int ini_leer_opcion(Ini *ini, const char *clave, const char *const opciones[], int n, int *idx)
{
    const IniEntrada *e = tomar(ini, clave);
    if (!e)
        return 1;
    for (int i = 0; i < n; ++i)
        if (!strcmp(e->texto, opciones[i])) {
            *idx = i;
            return 1;
        }
    fprintf(stderr, "%s: %s = '%s': se esperaba ", e->origen, clave, e->texto);
    for (int i = 0; i < n; ++i)
        fprintf(stderr, "%s%s", i ? " | " : "", opciones[i]);
    fprintf(stderr, "\n");
    ini->errores++;
    return 0;
}
//---------------------------------------------------------------------------
// Marca como usadas todas las claves de una seccion que este programa no lee (para que no se
// informen como desconocidas).
void ini_ignorar_seccion(Ini *ini, const char *seccion)
{
    const size_t n = strlen(seccion);
    for (int i = 0; i < ini->n; ++i)
        if (!strncmp(ini->e[i].clave, seccion, n) && ini->e[i].clave[n] == '.')
            ini->e[i].usada = 1;
}
//---------------------------------------------------------------------------
// Aplica cada clave de [seccion] como una asignacion "seccion.clave=valor" (sin seccion, de
// [general]) y la deja leida. La copia conserva el origen del texto: si nadie la pide, el mensaje
// dice donde se escribio.
void ini_superponer_seccion(Ini *ini, const char *seccion)
{
    const size_t n = strlen(seccion);
    const int n0 = ini->n; // las que agrega fijar() son de otras secciones
    for (int i = 0; i < n0; ++i) {
        IniEntrada *e = &ini->e[i];
        if (strncmp(e->clave, seccion, n) || e->clave[n] != '.')
            continue;
        const char *resto = e->clave + n + 1;
        char clave[160], texto[256], de[160];
        if (strchr(resto, '.'))
            copiar(clave, sizeof(clave), resto);
        else
            snprintf(clave, sizeof(clave), "general.%s", resto);
        copiar(texto, sizeof(texto), e->texto);
        copiar(de, sizeof(de), e->origen);
        e->usada = 1;
        fijar(ini, clave, texto, de);
    }
}
//---------------------------------------------------------------------------
// Informa las claves que nadie pidio (probablemente mal escritas) y devuelve cuantos errores
// hubo en total.
int ini_terminar(Ini *ini)
{
    for (int i = 0; i < ini->n; ++i)
        if (!ini->e[i].usada) {
            fprintf(stderr, "%s: clave desconocida '%s'\n", ini->e[i].origen, ini->e[i].clave);
            ini->errores++;
        }
    return ini->errores == 0;
}

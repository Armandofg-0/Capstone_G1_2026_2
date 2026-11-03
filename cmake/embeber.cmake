# Embebe un archivo como arreglo C: cmake -DENTRADA=<archivo> -DSALIDA=<.c> -P embeber.cmake
# Genera `const unsigned char pagina_html[]` y `const size_t pagina_html_n`. Los bytes van en
# hexadecimal porque MSVC limita el largo de un literal de cadena.
file(READ "${ENTRADA}" hex HEX)
string(LENGTH "${hex}" largo)
math(EXPR n "${largo} / 2")
string(REGEX REPLACE [[([0-9a-f][0-9a-f])]] [[0x\1,]] bytes "${hex}")
file(WRITE "${SALIDA}"
"/* Generado por cmake/embeber.cmake desde ${ENTRADA}. No editar. */
#include <stddef.h>
const unsigned char pagina_html[] = {
${bytes}0x00};
const size_t pagina_html_n = ${n};
")

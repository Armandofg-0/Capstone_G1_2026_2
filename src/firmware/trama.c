// COBS ("Consistent Overhead Byte Stuffing") es la forma de separar mensajes en un flujo de
// bytes: se recodifica el mensaje para que no contenga ningun byte 0x00 y se usa 0x00 como
// marca de fin. Si se pierde un byte, el receptor se resincroniza en el siguiente 0x00. Cuesta
// solo 1 byte extra cada 254. Ejemplo: [11 00 22] se codifica como [02 11 02 22 00]: cada
// "codigo" dice cuantos bytes faltan hasta donde habia un cero.
// Encuadre COBS (ver trama.h).
#include "trama.h"

// Codifica n bytes de msg en salida (que debe tener espacio para n + n/254 + 2) y devuelve el
// largo escrito, incluido el 0x00 final.
int trama_codificar(const uint8_t *msg, int n, uint8_t *salida)
{
    int k = 1, cod = 0; // cod: posicion del byte de codigo del bloque actual
    uint8_t cuenta = 1;
    for (int i = 0; i < n; ++i) {
        if (msg[i] == 0) {
            salida[cod] = cuenta;
            cod = k++;
            cuenta = 1;
        } else {
            salida[k++] = msg[i];
            if (++cuenta == 0xFF) {
                salida[cod] = cuenta;
                cod = k++;
                cuenta = 1;
            }
        }
    }
    salida[cod] = cuenta;
    salida[k++] = 0x00;
    return k;
}
//---------------------------------------------------------------------------
// Vacia el lector (descarta lo acumulado).
void trama_iniciar(LectorTrama *l)
{
    l->n = 0;
}
//---------------------------------------------------------------------------
// Recibe un byte a la vez. Mientras no llega 0x00 lo acumula; al llegar el 0x00 decodifica
// lo acumulado y, si es valido, deja el mensaje en l->salida y devuelve su largo. Una trama
// mal formada (o demasiado larga) se descarta y devuelve 0.
int trama_agregar(LectorTrama *l, uint8_t byte)
{
    if (byte != 0x00) {
        if (l->n < (int)sizeof(l->buf))
            l->buf[l->n++] = byte;
        else
            l->n = sizeof(l->buf) + 1; // desborde: se descarta hasta el proximo delimitador
        return 0;
    }
    const int n = l->n;
    l->n = 0;
    if (n == 0 || n > (int)sizeof(l->buf))
        return 0;
    int i = 0, k = 0;
    while (i < n) {
        const int cod = l->buf[i++];
        if (cod == 0 || i + cod - 1 > n)
            return 0;
        for (int j = 1; j < cod; ++j) {
            if (k >= TRAMA_MAX)
                return 0;
            l->salida[k++] = l->buf[i++];
        }
        if (cod < 0xFF && i < n) {
            if (k >= TRAMA_MAX)
                return 0;
            l->salida[k++] = 0;
        }
    }
    return k;
}

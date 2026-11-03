// Convierte los mensajes (plan Pi -> ESP32 y telemetria ESP32 -> Pi) entre su estructura en C
// y la secuencia de bytes que viaja por el cable, con un CRC al final. Los numeros se escriben
// byte a byte con el menos significativo primero ("little endian"), asi el formato no depende
// de la maquina que lo arme.
// Mensajes Pi <-> ESP32 (ver mensajes.h).
#include "mensajes.h"

#include <math.h>
#include <string.h>

#include "crc16.h"

// Ayudantes para escribir y leer enteros de 16 y 32 bits y reales float de 32 bits.
static void poner16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}
static void poner32(uint8_t *b, uint32_t v)
{
    for (int i = 0; i < 4; ++i)
        b[i] = (uint8_t)(v >> (8 * i));
}
static uint16_t leer16(const uint8_t *b)
{
    return (uint16_t)(b[0] | (b[1] << 8));
}
static uint32_t leer32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static void poner_f32(uint8_t *b, float f)
{
    uint32_t v;
    memcpy(&v, &f, 4);
    poner32(b, v);
}
static float leer_f32(const uint8_t *b)
{
    const uint32_t v = leer32(b);
    float f;
    memcpy(&f, &v, 4);
    return f;
}
//---------------------------------------------------------------------------
// Una velocidad no finita se manda como detencion: convertir NaN a un entero es comportamiento
// indefinido en C.
int16_t msg_velocidad_a_int16(double qd)
{
    if (isnan(qd))
        return 0;
    const double u = floor(qd / MSG_RES_VELOCIDAD + 0.5);
    return (int16_t)(u > 32767.0 ? 32767.0 : (u < -32767.0 ? -32767.0 : u));
}
//---------------------------------------------------------------------------
// Velocidad (rad/s) desde su codigo de 16 bits (resolucion MSG_RES_VELOCIDAD).
double msg_int16_a_velocidad(int16_t v)
{
    return v * MSG_RES_VELOCIDAD;
}
//---------------------------------------------------------------------------
// Plan: secuencia, t_a (cuando empieza a valer), M nodos optimos, N_ff nodos de cola, banderas
// y las velocidades de cada nodo para los dos ejes, mas el CRC. Devuelve el largo, o 0 si el
// plan tiene mas nodos de los que caben.
int msg_plan_codificar(const MsgPlan *p, uint8_t *buf)
{
    const int nodos = p->M + p->N_ff + 1;
    if (nodos > MSG_MAX_NODOS)
        return 0;
    poner16(buf, p->secuencia);
    poner32(buf + 2, p->t_a);
    buf[6] = p->M;
    buf[7] = p->N_ff;
    buf[8] = p->banderas;
    int k = 9;
    for (int i = 0; i < nodos; ++i)
        for (int j = 0; j < 2; ++j, k += 2)
            poner16(buf + k, (uint16_t)p->nodos[i][j]);
    poner16(buf + k, crc16(buf, (size_t)k));
    return k + 2;
}
//---------------------------------------------------------------------------
// Lo inverso: devuelve 1 solo si el largo cuadra con la cantidad de nodos y el CRC coincide.
int msg_plan_decodificar(const uint8_t *buf, int n, MsgPlan *p)
{
    if (n < 11 + 4)
        return 0;
    const int nodos = buf[6] + buf[7] + 1;
    if (nodos > MSG_MAX_NODOS || n != 11 + 4 * nodos || crc16(buf, (size_t)(n - 2)) != leer16(buf + n - 2))
        return 0;
    p->secuencia = leer16(buf);
    p->t_a = leer32(buf + 2);
    p->M = buf[6];
    p->N_ff = buf[7];
    p->banderas = buf[8];
    int k = 9;
    for (int i = 0; i < nodos; ++i)
        for (int j = 0; j < 2; ++j, k += 2)
            p->nodos[i][j] = (int16_t)leer16(buf + k);
    return 1;
}
//---------------------------------------------------------------------------
// Telemetria de 35 bytes: tick de la ESP32, pose y error estimados, que plan y nodo ejecuta,
// banderas y los instantes de recepcion (tau2) y envio (tau3) para sincronizar relojes.
int msg_telemetria_codificar(const MsgTelemetria *t, uint8_t *buf)
{
    poner32(buf, t->tau);
    poner_f32(buf + 4, t->q_hat[0]);
    poner_f32(buf + 8, t->q_hat[1]);
    poner_f32(buf + 12, t->e_hat[0]);
    poner_f32(buf + 16, t->e_hat[1]);
    poner16(buf + 20, t->secuencia);
    buf[22] = t->indice;
    buf[23] = t->edad;
    buf[24] = t->banderas;
    poner32(buf + 25, t->tau2);
    poner32(buf + 29, t->tau3);
    poner16(buf + 33, crc16(buf, 33));
    return MSG_TELEMETRIA_BYTES;
}
//---------------------------------------------------------------------------
// Lo inverso: devuelve 1 si el largo es 35 y el CRC coincide.
int msg_telemetria_decodificar(const uint8_t *buf, int n, MsgTelemetria *t)
{
    if (n != MSG_TELEMETRIA_BYTES || crc16(buf, 33) != leer16(buf + 33))
        return 0;
    t->tau = leer32(buf);
    t->q_hat[0] = leer_f32(buf + 4);
    t->q_hat[1] = leer_f32(buf + 8);
    t->e_hat[0] = leer_f32(buf + 12);
    t->e_hat[1] = leer_f32(buf + 16);
    t->secuencia = leer16(buf + 20);
    t->indice = buf[22];
    t->edad = buf[23];
    t->banderas = buf[24];
    t->tau2 = leer32(buf + 25);
    t->tau3 = leer32(buf + 29);
    return 1;
}
//---------------------------------------------------------------------------
// Comando de calibracion: marca 0xC0, orden, eje y valor.
int msg_comando_codificar(const MsgComando *c, uint8_t *buf)
{
    buf[0] = 0xC0;
    buf[1] = c->orden;
    buf[2] = c->eje;
    poner_f32(buf + 3, c->valor);
    poner16(buf + 7, crc16(buf, 7));
    return MSG_COMANDO_BYTES;
}
//---------------------------------------------------------------------------
int msg_comando_decodificar(const uint8_t *buf, int n, MsgComando *c)
{
    if (n != MSG_COMANDO_BYTES || buf[0] != 0xC0 || crc16(buf, 7) != leer16(buf + 7))
        return 0;
    c->orden = buf[1];
    c->eje = buf[2];
    c->valor = leer_f32(buf + 3);
    return 1;
}
//---------------------------------------------------------------------------
// Estado de calibracion: marca 0xC1, banderas, prueba de sentido y limites de los dos ejes.
int msg_estado_calib_codificar(const MsgEstadoCalib *e, uint8_t *buf)
{
    buf[0] = 0xC1;
    buf[1] = e->banderas;
    for (int j = 0; j < 2; ++j) {
        buf[2 + j] = e->sentido[j];
        poner_f32(buf + 4 + 4 * j, e->dq_sentido[j]);
        poner_f32(buf + 12 + 4 * j, e->q_min[j]);
        poner_f32(buf + 20 + 4 * j, e->q_max[j]);
    }
    poner16(buf + 28, crc16(buf, 28));
    return MSG_ESTADO_CALIB_BYTES;
}
//---------------------------------------------------------------------------
int msg_estado_calib_decodificar(const uint8_t *buf, int n, MsgEstadoCalib *e)
{
    if (n != MSG_ESTADO_CALIB_BYTES || buf[0] != 0xC1 || crc16(buf, 28) != leer16(buf + 28))
        return 0;
    e->banderas = buf[1];
    for (int j = 0; j < 2; ++j) {
        e->sentido[j] = buf[2 + j];
        e->dq_sentido[j] = leer_f32(buf + 4 + 4 * j);
        e->q_min[j] = leer_f32(buf + 12 + 4 * j);
        e->q_max[j] = leer_f32(buf + 20 + 4 * j);
    }
    return 1;
}

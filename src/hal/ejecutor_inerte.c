// El ejecutor inerte hace el papel de la ESP32 cuando no hay enlace serie (niveles 0 a 3):
// recibe los planes que manda el controlador, comprueba que lleguen bien armados y responde
// siempre con la misma telemetria, como si la torreta estuviera quieta en q0.
// Ejecutor inerte (ver ejecutor_inerte.h).
#include "ejecutor_inerte.h"

// Arranca en q0, sin planes contados y con el lector de tramas vacio.
void inerte_iniciar(EjecutorInerte *e, const double q0[2])
{
    e->q0[0] = q0[0];
    e->q0[1] = q0[1];
    trama_iniciar(&e->lector);
    e->planes_validos = e->planes_invalidos = 0;
    e->ultima_sec = 0;
    e->en_trama = 0;
}
//---------------------------------------------------------------------------
// Recibe los bytes de un plan tal como irian por el cable (tramas COBS terminadas en 0x00)
// y cuenta cuantos planes llegaron validos y cuantos no.
void inerte_recibir(EjecutorInerte *e, const uint8_t *bytes, int n)
{
    for (int i = 0; i < n; ++i) {
        const int delimitador = bytes[i] == 0x00;
        const int largo = trama_agregar(&e->lector, bytes[i]);
        if (!delimitador) {
            e->en_trama = 1;
            continue;
        }
        // trama_agregar devuelve 0 tambien al descartar una trama mal formada: el indicador
        // distingue ese caso de un delimitador suelto.
        const int habia = e->en_trama;
        e->en_trama = 0;
        if (largo <= 0) {
            if (habia)
                ++e->planes_invalidos;
            continue;
        }
        MsgPlan p;
        if (msg_plan_decodificar(e->lector.salida, largo, &p)) {
            ++e->planes_validos;
            e->ultima_sec = p.secuencia;
        } else {
            ++e->planes_invalidos;
        }
    }
}
//---------------------------------------------------------------------------
// Arma una telemetria fija: pose q0, sin error y la bandera "sin plan" (nunca ejecuta nada).
// "= {0}" deja todos los campos de la estructura en cero antes de llenar los que importan.
int inerte_telemetria(const EjecutorInerte *e, uint32_t tau_us, uint8_t msg[MSG_TELEMETRIA_BYTES])
{
    MsgTelemetria t = {0};
    t.tau = t.tau2 = t.tau3 = tau_us;
    t.q_hat[0] = (float)e->q0[0];
    t.q_hat[1] = (float)e->q0[1];
    t.banderas = TEL_SIN_PLAN;
    return msg_telemetria_codificar(&t, msg);
}

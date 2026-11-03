// Modelo de perdidas de paquetes en rafagas: el canal esta "bueno" o "malo", y pasa de uno a
// otro con probabilidades p (bueno -> malo) y q (malo -> bueno). Aqui estan el canal simulado y
// el monitor que estima p y q contando las perdidas reales.
// Gilbert-Elliott (ver gilbert_elliott.h).
#include "gilbert_elliott.h"

// Canal simulado con probabilidades p y q, arrancando en el estado bueno.
void ge_iniciar(CanalGE *c, double ge_p, double ge_q)
{
    c->ge_p = ge_p;
    c->ge_q = ge_q;
    c->perdiendo = 0;
}
//---------------------------------------------------------------------------
// Simula un paquete: u es un numero al azar entre 0 y 1. Devuelve 1 si se perdio.
int ge_perdido(CanalGE *c, double u)
{
    // Desde perdida se sigue perdiendo con probabilidad p; desde exito se pierde con 1 - q.
    c->perdiendo = c->perdiendo ? (u <= c->ge_p) : (u > c->ge_q);
    return c->perdiendo;
}
//---------------------------------------------------------------------------
// Monitor sin observaciones todavia.
void ge_monitor_iniciar(MonitorGE *m)
{
    m->n_P = m->n_PP = m->n_E = m->n_EE = 0;
    m->hay_anterior = m->anterior = 0;
}
//---------------------------------------------------------------------------
// Registra si un paquete real se perdio o llego, contando las transiciones entre estados.
void ge_monitor_registrar(MonitorGE *m, int perdido)
{
    if (m->hay_anterior) {
        if (m->anterior) {
            m->n_P++;
            m->n_PP += perdido;
        } else {
            m->n_E++;
            m->n_EE += !perdido;
        }
    }
    m->anterior = perdido;
    m->hay_anterior = 1;
}
//---------------------------------------------------------------------------
// Estimaciones de p y q con lo contado hasta ahora.
double ge_monitor_p(const MonitorGE *m)
{
    return m->n_P ? (double)m->n_PP / m->n_P : 0.0;
}
//---------------------------------------------------------------------------
double ge_monitor_q(const MonitorGE *m)
{
    return m->n_E ? (double)m->n_EE / m->n_E : 1.0;
}
//---------------------------------------------------------------------------
// Margen de estabilidad: cuanto falta para violar p * gamma < 1 (negativo = se violo).
double ge_monitor_margen(const MonitorGE *m, double gamma)
{
    return 1.0 - ge_monitor_p(m) * gamma;
}

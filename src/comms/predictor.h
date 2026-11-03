// Predictor Phi (perdida.tex, "Aplicacion programada adaptativa"):
//   x0 = x_hat(t_a) = Phi(x_hat(tau_tel), C_k, tau_tel -> t_a).
// Reproduce en la Pi la logica determinista del ejecutor: un espejo del nucleo (el mismo
// codigo de src/firmware/ejecutor) se resincroniza con cada telemetria y se avanza, ciclo a
// ciclo, con los paquetes comprometidos C_k (los enviados que el ejecutor tendra hasta t_a).
//
// Resincronizacion con la telemetria del tick tau_tel:
//  - q_cmd = q_hat + e_hat (exacto, salvo la precision float32);
//  - el plan vigente es el enviado con la secuencia que informa la ESP32 (si se perdio un
//    paquete, lo revela aqui); los posteriores se suponen entregados;
//  - q_dot_cmd es la consigna de ese plan en tau_tel, salvo que la ESP32 informe limitador o
//    frenado: entonces se conserva la del espejo, que vivio las mismas perdidas.
#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "ejecutor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PHI_ENVIADOS 32

typedef struct {
    NucleoEjecutor espejo;  // estado del ejecutor en tau_tel
    uint32_t tau_tel;
    int hay_tel;
    MsgPlan enviados[PHI_ENVIADOS];
    int n_env;
} Predictor;

void phi_iniciar(Predictor *f, const ConfigEjecutor *cfg, const double q0[2]);

// Registra un plan enviado (comprometido).
void phi_enviado(Predictor *f, const MsgPlan *p);

// Resincroniza con una telemetria recibida.
void phi_telemetria(Predictor *f, const MsgTelemetria *t);

// Estado comandado x = [q_cmd; q_dot_cmd] que tendra el ejecutor despues de su ciclo del tick
// t_a (t_a debe caer en un ciclo del ejecutor: tau_tel + k dt). Devuelve 0 si todavia no
// llego ninguna telemetria (en ese caso, x es el estado inicial).
int phi_predecir(const Predictor *f, uint32_t t_a, double x[4]);

#ifdef __cplusplus
}
#endif

#endif /* PREDICTOR_H */

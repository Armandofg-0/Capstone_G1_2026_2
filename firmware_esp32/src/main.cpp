// Ejecutor de la ESP32 (perdida.tex, "El ejecutor de la ESP32"): drivers de esta placa
// alrededor del nucleo portable (src/firmware/ejecutor.h).
//
//  - Tarea del ejecutor (nucleo 1, 1 kHz): lee los AS5600, corre nucleo_ciclo y fija la
//    posicion objetivo de cada driver en micropasos; cada 1/F_TELEMETRIA envia la telemetria.
//  - Timer de 20 kHz: lleva la posicion del driver hacia el objetivo, un micropaso por flanco
//    (DRV8825: pulso de 50 us, DIR fijado un periodo antes del flanco).
//  - Tarea de recepcion (nucleo 0): bytes de la UART2 -> COBS -> nucleo_recibir con tau2.
//  - Cada 10 ms lee los INA219; una sobrecorriente sostenida deshabilita los drivers (EN alto)
//    hasta reiniciar la placa.
//  - Cero, limites y ultima pose persistentes (NVS). El AS5600 va en el motor y es absoluto en
//    una vuelta del motor, que es 1/RELACION[j] de vuelta del eje: al arrancar se eligen las
//    vueltas del motor que dejan la pose mas cerca de la ultima guardada (guardar_pose). Sin calibracion guardada, o con un eje fuera de
//    [q_min, q_max], arranca en modo calibracion (consola USB, ver consola()). Un encoder fuera
//    de los limites mas MARGEN_LIMITE deshabilita los drivers hasta reiniciar la placa.
//
// En pocas palabras: este archivo conecta el nucleo (la logica, en src/firmware) con el
// hardware de la placa: pines, buses I2C, UART, timer y memoria flash. Corre sobre FreeRTOS,
// el sistema operativo de la ESP32, que permite tener varias "tareas" (hilos) a la vez y
// repartirlas entre sus dos nucleos.
//
//  - Con TILT_ACTIVO = 0 (tilt sin motor) el tilt no se mueve ni se
//    lee: su driver queda apagado, su q es TILT_FIJO y no entra en limites, corriente,
//    calibracion ni pose guardada. La Pi le manda velocidad cero.
//
// tau es micros() (32 bits, us), el mismo tick que usa el nucleo.
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <math.h>

#include "config.h"
#include "ejecutor.h"
#include "mensajes.h"
#include "trama.h"

namespace {

const uint8_t AS5600 = 0x36, AS5600_ANGULO = 0x0C;
const uint8_t INA219 = 0x40, INA219_SHUNT = 0x01;

// Los dos buses I2C (Wire y Wire1), uno por eje: cada eje tiene su encoder y su sensor de
// corriente con la misma direccion, por eso van en buses separados.
TwoWire *bus[2] = {&Wire, &Wire1};
// Ejes con motor: el pan siempre; el tilt segun [planta] tilt_activo.
const bool ACTIVO[2] = {true, TILT_ACTIVO != 0};
NucleoEjecutor nucleo;
// El nucleo lo usan dos tareas (recepcion y ejecutor) en nucleos distintos de la CPU. Un
// portMUX es un candado: entre portENTER_CRITICAL y portEXIT_CRITICAL solo una tarea puede
// tocar el nucleo, asi nunca lo ve a medio actualizar.
portMUX_TYPE candado = portMUX_INITIALIZER_UNLOCKED; // nucleo compartido entre tareas

// Estado del generador de pasos (lo escribe la tarea, lo lee el timer).
// "volatile" avisa al compilador que la variable puede cambiar "por fuera" (aqui, desde la
// interrupcion del timer), asi que debe leerla de memoria cada vez y no guardarla en un
// registro.
volatile long objetivo[2] = {0, 0};
volatile long posicion[2] = {0, 0};
volatile bool habilitado = true;

// Calibracion: con calibrando el nucleo no corre y se ignoran las consignas de la Pi.
ConfigEjecutor cfg;
volatile bool calibrando = false, pedir_cero = false;
bool calibracion_guardada = false; // hay cero y limites en la NVS
// Ultima prueba de sentido de cada eje (SENTIDO_*) y lo que midio el encoder (grados).
uint8_t sentido[2] = {SENTIDO_NINGUNO, SENTIDO_NINGUNO};
float dq_sentido[2] = {0.0f, 0.0f};
// Comando de calibracion recibido por la UART: lo deja la tarea de recepcion y lo ejecuta loop(),
// porque mover un eje bloquea. hay_comando se escribe al final, dentro del candado.
portMUX_TYPE candado_comando = portMUX_INITIALIZER_UNLOCKED;
MsgComando comando_pendiente;
volatile bool hay_comando = false;
volatile double q_med[2] = {0.0, 0.0};

// Encoder (en el motor): ultima lectura cruda, vueltas del motor acumuladas y cero al arrancar.
int crudo_previo[2], vueltas[2] = {0, 0}, crudo_cero[2];
// Ultima pose guardada en la NVS, en cuentas del motor desde el cero (ver guardar_pose).
int pose_guardada[2] = {0, 0};

// Sensores que respondieron al arrancar: a uno ausente no se lo interroga (cada intento fallido
// cuesta el timeout del bus y un mensaje de error de Wire).
bool hay_enc[2] = {false, false}, hay_ina[2] = {false, false};
// Lecturas fallidas desde la ultima telemetria (las informa TEL_FALLA_ENCODER / TEL_FALLA_INA).
bool falla_enc = false, falla_ina = false;

// Pregunta si hay un dispositivo en esa direccion del bus: se le habla sin mandar datos y se
// mira si contesta (endTransmission devuelve 0 si respondio).
bool responde(int j, uint8_t dir)
{
    bus[j]->beginTransmission(dir);
    return bus[j]->endTransmission() == 0;
}

// Lee un registro de 16 bits de un dispositivo I2C: primero se le dice que registro (write) y
// luego se piden 2 bytes (requestFrom). El "false" mantiene el bus tomado entre ambos pasos.
bool leer16(int j, uint8_t dir, uint8_t reg, uint16_t &v)
{
    bus[j]->beginTransmission(dir);
    bus[j]->write(reg);
    if (bus[j]->endTransmission(false) != 0 || bus[j]->requestFrom(dir, (uint8_t)2) != 2) return false;
    v = (uint16_t)(bus[j]->read() << 8);
    v |= (uint16_t)bus[j]->read();
    return true;
}

// Angulo del eje j (rad) desde el arranque, desenvuelto. Si la lectura falla, repite la ultima.
double angulo(int j)
{
    if (!ACTIVO[j]) return TILT_FIJO;
    uint16_t v;
    if (hay_enc[j] && leer16(j, AS5600, AS5600_ANGULO, v)) {
        const int c = v & 0x0FFF;
        const int d = c - crudo_previo[j];
        if (d > 2048) vueltas[j]--;
        else if (d < -2048) vueltas[j]++;
        crudo_previo[j] = c;
    } else {
        falla_enc = true;
    }
    const double cuentas = 4096.0 * vueltas[j] + crudo_previo[j] - crudo_cero[j];
    return SIGNO_ENC[j] * cuentas * (2.0 * M_PI / 4096.0) / RELACION[j];
}

// Corriente del driver j (A), por la tension del shunt (LSB 10 uV). NAN si falla.
double corriente(int j)
{
    uint16_t v;
    if (!hay_ina[j] || !leer16(j, INA219, INA219_SHUNT, v)) {
        falla_ina = true;
        return NAN;
    }
    return (int16_t)v * 1e-5 / R_SHUNT;
}

// Apaga los drivers (EN en alto) y detiene la generacion de pasos. Se usa ante una parada de
// emergencia de la Pi (CMD_PARAR) y ante sobrecorriente
// o si un eje se sale de sus limites; solo se recupera reiniciando la placa.
void deshabilitar()
{
    habilitado = false;
    for (int j = 0; j < 2; ++j) digitalWrite(PIN_EN[j], HIGH);
}

// Interrupcion del timer, 20 000 veces por segundo. IRAM_ATTR la deja en la RAM interna para
// que corra rapido y sin depender de la flash. En cada llamada, si la posicion del driver no
// coincide con el objetivo, da un micropaso (flanco de subida en STEP) en esa direccion; en la
// llamada siguiente baja el pulso. Si hay que cambiar de sentido, primero cambia DIR y espera
// una llamada, como pide el DRV8825.
void IRAM_ATTR pulso()
{
    static bool alto[2] = {false, false};
    if (!habilitado) return;
    for (int j = 0; j < 2; ++j) {
        if (!ACTIVO[j]) continue;
        if (alto[j]) { // fin del pulso: cuenta el micropaso
            digitalWrite(PIN_STEP[j], LOW);
            alto[j] = false;
            continue;
        }
        const long e = objetivo[j] - posicion[j];
        if (e == 0) continue;
        const int sentido = e > 0 ? 1 : -1;
        const int nivel_dir = sentido * SIGNO_DIR[j] > 0 ? HIGH : LOW;
        if (digitalRead(PIN_DIR[j]) != nivel_dir) { // cambio de sentido: DIR primero
            digitalWrite(PIN_DIR[j], nivel_dir);
            continue;
        }
        digitalWrite(PIN_STEP[j], HIGH);
        alto[j] = true;
        posicion[j] += sentido;
    }
}

// Fija la resolucion de micropaso del DRV8825 con los pines M0 y M1 (M2 va a tierra).
void configurar_micropasos(int j)
{
    // DRV8825 con M2 = 0: paso completo (ninguno), 1/2 (M0), 1/4 (M1), 1/8 (M0 y M1).
    digitalWrite(PIN_M0[j], MICROPASOS[j] == 2 || MICROPASOS[j] == 8 ? HIGH : LOW);
    digitalWrite(PIN_M1[j], MICROPASOS[j] == 4 || MICROPASOS[j] == 8 ? HIGH : LOW);
}

// Tarea principal, cada 1 ms en el nucleo 1 de la CPU: lee los encoders, revisa la barrera de
// limites, corre un ciclo del nucleo y pasa la posicion objetivo al generador de pasos. Cada
// tanto envia la telemetria a la Pi y revisa la corriente. vTaskDelayUntil mantiene el periodo
// exacto aunque el trabajo de cada vuelta dure distinto. El watchdog reinicia la placa si esta
// tarea se cuelga.
void tarea_ejecutor(void *)
{
    const int cada_tel = (int)lround(1.0 / (F_TELEMETRIA * DT_EJECUTOR));
    const int cada_ina = (int)lround(0.01 / DT_EJECUTOR);
    int sobre = 0;
    uint8_t msg[MSG_TELEMETRIA_BYTES], tr[2 * MSG_TELEMETRIA_BYTES];
    esp_task_wdt_add(nullptr); // si la tarea se cuelga 1 s, la placa se reinicia
    TickType_t despertar = xTaskGetTickCount();
    for (unsigned long tick = 0;; ++tick) {
        vTaskDelayUntil(&despertar, pdMS_TO_TICKS(1));
        esp_task_wdt_reset();
        if (pedir_cero) {
            for (int j = 0; j < 2; ++j) {
                crudo_cero[j] = crudo_previo[j];
                vueltas[j] = 0;
            }
            pedir_cero = false;
        }
        const double q_enc[2] = {angulo(0), angulo(1)};
        q_med[0] = q_enc[0];
        q_med[1] = q_enc[1];
        const uint32_t tau = micros();
        MsgTelemetria t;
        const bool tel = tick % cada_tel == 0;
        if (!calibrando) {
            for (int j = 0; j < 2; ++j) {
                if (habilitado && ACTIVO[j] && (q_enc[j] < cfg.q_min[j] - MARGEN_LIMITE || q_enc[j] > cfg.q_max[j] + MARGEN_LIMITE)) {
                    deshabilitar();
                    Serial.printf("eje %d fuera de limites (%.3f rad); drivers deshabilitados\n", j, q_enc[j]);
                }
            }
            portENTER_CRITICAL(&candado);
            nucleo_ciclo(&nucleo, tau, q_enc);
            for (int j = 0; j < 2; ++j) objetivo[j] = nucleo.pasos[j];
            if (tel) nucleo_telemetria(&nucleo, tau, &t);
            portEXIT_CRITICAL(&candado);
        }
        if (tel && calibrando) {
            // Sin nucleo: la pose es la del encoder y no hay plan en ejecucion.
            memset(&t, 0, sizeof t);
            t.tau = tau;
            t.q_hat[0] = (float)q_enc[0];
            t.q_hat[1] = (float)q_enc[1];
            t.banderas = TEL_SIN_PLAN;
        }
        if (tel) {
            if (!habilitado) t.banderas |= TEL_DRIVERS_OFF;
            if (falla_enc) t.banderas |= TEL_FALLA_ENCODER;
            if (falla_ina) t.banderas |= TEL_FALLA_INA;
            falla_enc = falla_ina = false;
            t.tau3 = micros();
            msg_telemetria_codificar(&t, msg);
            Serial2.write(tr, trama_codificar(msg, MSG_TELEMETRIA_BYTES, tr));
        }
        if (tick % 100 == 0) { // estado de calibracion a 10 Hz
            MsgEstadoCalib e;
            e.banderas = (calibrando ? CALIB_CALIBRANDO : 0) | (calibracion_guardada ? CALIB_GUARDADA : 0) |
                         (hay_enc[0] ? CALIB_ENC0 : 0) | (hay_enc[1] ? CALIB_ENC1 : 0) |
                         (hay_ina[0] ? CALIB_INA0 : 0) | (hay_ina[1] ? CALIB_INA1 : 0);
            for (int j = 0; j < 2; ++j) {
                e.sentido[j] = sentido[j];
                e.dq_sentido[j] = dq_sentido[j];
                e.q_min[j] = (float)cfg.q_min[j];
                e.q_max[j] = (float)cfg.q_max[j];
            }
            uint8_t m[MSG_ESTADO_CALIB_BYTES], te[2 * MSG_ESTADO_CALIB_BYTES];
            msg_estado_calib_codificar(&e, m);
            Serial2.write(te, trama_codificar(m, MSG_ESTADO_CALIB_BYTES, te));
        }
        if (tick % cada_ina == 0 && habilitado) {
            const double i0 = corriente(0), i1 = ACTIVO[1] ? corriente(1) : 0.0;
            sobre = (fabs(i0) > I_MAX || fabs(i1) > I_MAX) ? sobre + 1 : 0;
            if (sobre >= N_SOBRECORRIENTE) {
                deshabilitar();
                Serial.printf("sobrecorriente: %.2f A, %.2f A; drivers deshabilitados\n", i0, i1);
            }
        }
    }
}

// Tarea de recepcion, en el nucleo 0 de la CPU: lee los bytes que llegan de la Pi, arma las
// tramas COBS y entrega cada plan al nucleo. Durante la calibracion los planes se ignoran.
void tarea_recepcion(void *)
{
    static LectorTrama lector;
    trama_iniciar(&lector);
    for (;;) {
        while (Serial2.available()) {
            const int n = trama_agregar(&lector, (uint8_t)Serial2.read());
            MsgComando c;
            if (n > 0 && msg_comando_decodificar(lector.salida, n, &c)) {
                if (c.orden == CMD_PARAR) { // inmediata: no espera a loop()
                    deshabilitar();
                    continue;
                }
                if (c.orden == CMD_REINICIAR) ESP.restart();
                portENTER_CRITICAL(&candado_comando);
                comando_pendiente = c;
                hay_comando = true;
                portEXIT_CRITICAL(&candado_comando);
                continue;
            }
            if (n <= 0 || calibrando) continue;
            const uint32_t tau2 = micros();
            portENTER_CRITICAL(&candado);
            nucleo_recibir(&nucleo, lector.salida, n, tau2);
            portEXIT_CRITICAL(&candado);
        }
        vTaskDelay(1);
    }
}

// El nucleo parte de la pose medida; el generador de pasos se rebasa a ella sin moverse.
void iniciar_nucleo()
{
    const double q0[2] = {q_med[0], q_med[1]};
    portENTER_CRITICAL(&candado);
    nucleo_iniciar(&nucleo, &cfg, q0);
    for (int j = 0; j < 2; ++j) objetivo[j] = posicion[j] = nucleo.pasos[j];
    portEXIT_CRITICAL(&candado);
}

// true si los ejes con motor estan dentro de [q_min, q_max].
bool dentro_de_limites()
{
    for (int j = 0; j < 2; ++j)
        if (ACTIVO[j] && q_med[j] < cfg.q_min[j] || q_med[j] > cfg.q_max[j]) return false;
    return true;
}

// Mueve el eje j en dq (rad) a QD_MAX/4, un micropaso a la vez.
void trotar(int j, double dq)
{
    const long n = lround(fabs(dq) / cfg.micropaso[j]);
    const uint32_t periodo = (uint32_t)(cfg.micropaso[j] / (QD_MAX / 4.0) * 1e6);
    for (long i = 0; i < n && habilitado; ++i) {
        objetivo[j] += dq > 0.0 ? 1 : -1;
        delayMicroseconds(periodo);
    }
}

// Estado de la comunicacion con los sensores I2C de cada eje (detectados al arrancar).
void informar_sensores()
{
    for (int j = 0; j < 2; ++j)
        if (!ACTIVO[j])
            Serial.printf("eje %d (tilt): sin motor, fijo en %.4f rad\n", j, TILT_FIJO);
        else
            Serial.printf("eje %d (%s): AS5600 %s, INA219 %s\n", j, j == 0 ? "pan" : "tilt",
                          hay_enc[j] ? "ok" : "NO RESPONDE", hay_ina[j] ? "ok" : "NO RESPONDE");
}

// Prueba de sentido del eje j: lo mueve PRUEBA_GRADOS en positivo y lo devuelve. Segun la cadena
// R(q) del informe, +pan gira a la izquierda (visto desde atras de la camara) y +tilt sube la
// camara. El sentido fisico solo lo puede juzgar quien mira; el firmware solo compara el signo
// del encoder con el del comando.
void probar_sentido(int j)
{
    const double PRUEBA_GRADOS = 10.0, a = PRUEBA_GRADOS * M_PI / 180.0;
    const char *positivo = j == 0 ? "a la IZQUIERDA (visto desde atras de la camara)" : "hacia ARRIBA";
    const char *negativo = j == 0 ? "a la derecha" : "hacia abajo";
    if (q_med[j] + a > cfg.q_max[j] || q_med[j] < cfg.q_min[j]) {
        sentido[j] = SENTIDO_SIN_ESPACIO;
        Serial.printf("eje %d: sin espacio para +%.0f grados dentro de los limites; acercalo al centro con j\n", j,
                      PRUEBA_GRADOS);
        return;
    }
    const double q0 = q_med[j];
    Serial.printf("eje %d: +%.0f grados, debe girar %s\n", j, PRUEBA_GRADOS, positivo);
    trotar(j, a);
    delay(300); // que el eje se asiente y el encoder se actualice
    const double dq = q_med[j] - q0;
    Serial.printf("eje %d: vuelve, debe girar %s\n", j, negativo);
    trotar(j, -a);
    dq_sentido[j] = (float)(dq * 180.0 / M_PI);
    sentido[j] = !hay_enc[j] ? SENTIDO_SIN_ENCODER
                 : (dq * a <= 0.0 || fabs(dq) < 0.5 * a) ? SENTIDO_NO_ACOMPANA : SENTIDO_OK;
    if (hay_enc[j]) {
        Serial.printf("eje %d: el encoder midio %+.1f grados\n", j, dq * 180.0 / M_PI);
        if (sentido[j] == SENTIDO_NO_ACOMPANA)
            Serial.printf("ADVERTENCIA eje %d: el encoder no acompana al comando; revisa el iman o invierte "
                          "signo_enc en [esp32] de config/torreta.ini\n", j);
    }
    Serial.printf("Si el eje %d giro al reves, cambia las conexiones de una bobina del motor (o invierte "
                  "signo_dir en [esp32] de config/torreta.ini)\n", j);
}

// Guarda el cero y los limites en la NVS: una zona de la flash que se conserva al apagar.
// Preferences es la libreria de Arduino para usarla como un diccionario clave -> valor.
void guardar()
{
    Preferences nvs;
    nvs.begin("torreta", false);
    nvs.putInt("cero0", crudo_cero[0]);
    nvs.putInt("cero1", crudo_cero[1]);
    nvs.putDouble("qmin0", cfg.q_min[0]);
    nvs.putDouble("qmax0", cfg.q_max[0]);
    nvs.putDouble("qmin1", cfg.q_min[1]);
    nvs.putDouble("qmax1", cfg.q_max[1]);
    nvs.putBool("ok", true);
    nvs.end();
}

// Cuentas del motor del eje j desde el cero.
int cuentas_motor(int j) { return 4096 * vueltas[j] + crudo_previo[j] - crudo_cero[j]; }

// Guarda la pose de cada eje en la NVS cuando quedo quieta 1 s lejos de la ultima guardada
// (mas de 1/16 de vuelta del motor). Asi, al apagar con el eje detenido, el arranque sabe en
// que vuelta del motor esta, y la flash solo se escribe al detenerse, no mientras se sigue al
// blanco. Corre en loop(): la escritura demora la tarea del ejecutor unos ms, pero no al
// timer de pasos (pulso esta en IRAM).
void guardar_pose()
{
    static unsigned long t_ultimo = 0;
    static int previa[2] = {0, 0}, quieto[2] = {0, 0};
    if (millis() - t_ultimo < 250) return;
    t_ultimo = millis();
    for (int j = 0; j < 2; ++j) {
        if (!ACTIVO[j]) continue;
        const int c = cuentas_motor(j);
        quieto[j] = abs(c - previa[j]) < 16 ? quieto[j] + 1 : 0;
        previa[j] = c;
        if (quieto[j] >= 4 && abs(c - pose_guardada[j]) > 256) {
            Preferences nvs;
            nvs.begin("torreta", false);
            nvs.putInt(j == 0 ? "pose0" : "pose1", c);
            nvs.end();
            pose_guardada[j] = c;
        }
    }
}

// Una orden de calibracion (c, z, n, x, j, d, g o q), de la consola o de la Pi. Devuelve nullptr
// si se hizo, o el motivo por el que no. e es el eje (-1 si la orden no lo usa).
const char *calibrar(char orden, int e, double grados)
{
    const bool eje = (e == 0 || e == 1) && ACTIVO[e];
    if (orden == 'c') {
        calibrando = true;
        for (int j = 0; j < 2; ++j) objetivo[j] = posicion[j];
        return nullptr;
    }
    if (!calibrando) return "primero c";
    if (orden == 'z') {
        pedir_cero = true;
    } else if (orden == 'n' && eje) {
        cfg.q_min[e] = q_med[e];
    } else if (orden == 'x' && eje) {
        cfg.q_max[e] = q_med[e];
    } else if (orden == 'j' && eje) {
        trotar(e, grados * M_PI / 180.0);
    } else if (orden == 'd' && eje) {
        if (!(cfg.q_min[e] < 0.0 && cfg.q_max[e] > 0.0)) return "primero fija los extremos del eje (n y x)";
        probar_sentido(e);
    } else if (orden == 'g' || orden == 'q') {
        // g guarda y sale; q sale sin guardar. En ambos casos la pose debe quedar dentro de los
        // limites, porque el nucleo frena en ellos.
        for (int j = 0; j < 2; ++j)
            if (ACTIVO[j] && !(cfg.q_min[j] < 0.0 && cfg.q_max[j] > 0.0)) return "limites invalidos";
        if (!dentro_de_limites()) return "pose fuera de limites";
        if (orden == 'g') {
            guardar();
            calibracion_guardada = true;
        }
        iniciar_nucleo();
        calibrando = false;
    } else {
        return "orden desconocida";
    }
    return nullptr;
}

// Ejecuta el comando de la Pi que dejo la tarea de recepcion, si hay uno.
void atender_comando()
{
    if (!hay_comando) return;
    portENTER_CRITICAL(&candado_comando);
    const MsgComando c = comando_pendiente;
    hay_comando = false;
    portEXIT_CRITICAL(&candado_comando);
    if (c.orden < 1 || c.orden > CMD_SENTIDO || c.orden == CMD_PARAR || c.orden == CMD_REINICIAR) return;
    const char orden = c.orden == CMD_SENTIDO ? 'd' : "\0cqjznxg"[c.orden]; // indice = CMD_*
    const char *error = calibrar(orden, c.eje, c.valor);
    if (error) Serial.printf("comando de la Pi %d: %s\n", c.orden, error);
}

// Consola USB, una orden por linea:
//   c         entra al modo calibracion       z    cero en la pose actual (ambos ejes)
//   j E GRAD  mueve el eje E (0 pan, 1 tilt)  n E  q_min del eje E = pose actual
//   p         muestra pose y limites          x E  q_max del eje E = pose actual
//   s         estado de los sensores          d E  prueba de sentido del eje E (tras n y x)
//   g         guarda en NVS y vuelve a operar (exige q_min < 0 < q_max y la pose dentro)
//   q         vuelve a operar sin guardar (mismas exigencias)
// La Pi manda las mismas ordenes por la UART (MsgComando, ver atender_comando).
void consola()
{
    if (!Serial.available()) return;
    const String linea = Serial.readStringUntil('\n');
    char orden = 0;
    int e = -1;
    double grados = 0.0;
    const int leidos = sscanf(linea.c_str(), " %c %d %lf", &orden, &e, &grados);
    if (leidos < 1) return;
    if (orden == 'p') {
        Serial.printf("q = (%.4f, %.4f) rad; pan [%.4f, %.4f], tilt [%.4f, %.4f]; %s\n", q_med[0], q_med[1],
                      cfg.q_min[0], cfg.q_max[0], cfg.q_min[1], cfg.q_max[1], calibrando ? "calibrando" : "operando");
        return;
    }
    if (orden == 's') {
        informar_sensores();
        return;
    }
    if (orden == 'j' && leidos < 3) {
        Serial.println("j E GRAD");
        return;
    }
    const char *error = calibrar(orden, leidos >= 2 ? e : -1, grados);
    Serial.println(error ? error : "ok");
}

} // namespace

// setup() lo llama Arduino una vez al encender: configura pines, buses y UART, lee la
// calibracion guardada, arranca el nucleo, el timer de pasos y las dos tareas.
void setup()
{
    Serial.begin(115200);
    for (int j = 0; j < 2; ++j) {
        pinMode(PIN_EN[j], OUTPUT);
        digitalWrite(PIN_EN[j], HIGH);
        for (int p : {PIN_M0[j], PIN_M1[j], PIN_STEP[j], PIN_DIR[j]}) {
            pinMode(p, OUTPUT);
            digitalWrite(p, LOW);
        }
        configurar_micropasos(j);
        if (!ACTIVO[j]) continue; // driver apagado (EN alto) y sin bus I2C
        bus[j]->begin(PIN_SDA[j], PIN_SCL[j], 400000);
        bus[j]->setTimeOut(2); // ms: un sensor que falla no debe frenar la tarea de 1 kHz
        hay_enc[j] = responde(j, AS5600);
        hay_ina[j] = responde(j, INA219);
    }
    informar_sensores(); // sin AS5600 la q del eje queda fija; sin INA219 no hay corte por corriente
    Preferences nvs;
    nvs.begin("torreta", true);
    const bool calibrado = nvs.getBool("ok", false);
    calibracion_guardada = calibrado;
    for (int j = 0; j < 2; ++j) {
        if (!ACTIVO[j]) {
            q_med[j] = TILT_FIJO;
            continue;
        }
        uint16_t v = 0;
        leer16(j, AS5600, AS5600_ANGULO, v);
        crudo_previo[j] = v & 0x0FFF;
        crudo_cero[j] = nvs.getInt(j == 0 ? "cero0" : "cero1", crudo_previo[j]);
        // Vueltas del motor que dejan la pose mas cerca de la ultima guardada (sin pose
        // guardada, de 0: con RELACION 1 es el caso de un recorrido que cabe en una vuelta).
        pose_guardada[j] = nvs.getInt(j == 0 ? "pose0" : "pose1", 0);
        const int d = crudo_previo[j] - crudo_cero[j];
        vueltas[j] = (int)lround((pose_guardada[j] - d) / 4096.0);
        q_med[j] = angulo(j);
    }
    Serial2.setRxBufferSize(1024);
    Serial2.begin(BAUDIOS, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

    ConfigEjecutor &c = cfg;
    c.Ts = TS;
    c.dt = DT_EJECUTOR;
    c.a_lim = A_LIM;
    c.a_frenado = A_FRENADO;
    c.q_min[0] = nvs.getDouble("qmin0", Q_MIN_PAN);
    c.q_max[0] = nvs.getDouble("qmax0", Q_MAX_PAN);
    c.q_min[1] = nvs.getDouble("qmin1", Q_MIN_TILT);
    c.q_max[1] = nvs.getDouble("qmax1", Q_MAX_TILT);
    nvs.end();
    c.margen_limite = MARGEN_LIMITE;
    c.qd_max = QD_MAX;
    for (int j = 0; j < 2; ++j) c.micropaso[j] = 2.0 * M_PI / (PASOS_VUELTA * MICROPASOS[j] * RELACION[j]);
    c.kp = KP;
    c.tau_correccion = TAU_CORRECCION;
    c.alfa = ALFA;
    c.beta = BETA;
    iniciar_nucleo();
    calibrando = !calibrado || !dentro_de_limites();
    if (calibrando) Serial.println("modo calibracion: c, z, j E GRAD, n E, x E, d E, s, p, g, q");

    for (int j = 0; j < 2; ++j)
        if (ACTIVO[j]) digitalWrite(PIN_EN[j], LOW);
    hw_timer_t *timer = timerBegin(0, 80, true); // 1 us por cuenta
    timerAttachInterrupt(timer, pulso, true);
    timerAlarmWrite(timer, 50, true);            // 20 kHz
    timerAlarmEnable(timer);

    esp_task_wdt_init(1, true); // plazo minimo de la API de IDF 4 (s)
    xTaskCreatePinnedToCore(tarea_ejecutor, "ejecutor", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
    xTaskCreatePinnedToCore(tarea_recepcion, "recepcion", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
}

// loop() lo llama Arduino una y otra vez; aqui atiende la consola USB y guarda la pose, con prioridad
// baja, mientras las tareas hacen el trabajo de tiempo real.
void loop()
{
    consola();
    atender_comando();
    guardar_pose();
}

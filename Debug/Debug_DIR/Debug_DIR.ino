// ============================================================
//  Diagnóstico del pin DIR en los dos DRV8825
// ============================================================
//  Para cada motor se hacen dos pruebas:
//
//  1) Estática (motor quieto): DIR se deja fijo en LOW y luego en HIGH.
//     En cada nivel se imprime:
//       - escrito: lo que la ESP32 manda a la salida (GPIO_OUT_REG)
//       - pin: lo que realmente hay en el pad (GPIO_IN_REG)
//
//  2) Dinámica: se dan PASOS pasos con DIR en LOW y luego en HIGH, para
//     observar si el eje invierte el sentido.
//
//  Cómo interpretar el resultado (ver también el resumen al final):
//    - escrito cambia y pin NO cambia: algo externo fuerza el pin: corto en la PCB a GND o a 3,3 V,
//           o el pad de la ESP32 está dañado.

//    - escrito y pin cambian, pero el multímetro NO ve el cambio en el
//      pin DIR del DRV8825: la ESP32 funciona; la falla está entre la ESP32 y el driver
//           (pista, soldadura, header o pin equivocado).

//    - el cambio SÍ llega al pin DIR del driver, pero el motor no invierte: la ESP32 y la PCB funcionan; revisar el driver o el cableado
//           de las bobinas del motor.
// ============================================================
#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

const int PASOS      = 400;
const int PERIODO_US = 2500;
const int PULSO_US   = 5;
const int STEP_PIN[2]  = {26, 17};
const int DIR_PIN[2]   = {25, 16};
const int EN_PIN[2]    = {12, 19};
const int MODE0_PIN[2] = {14, 23};
const int MODE1_PIN[2] = {27, 18};

// ============================================================
// FUNCIONES
// ============================================================

// Nivel que la ESP32 manda a la salida (registro de salida).
int nivelEscrito(int pin) { return (REG_READ(GPIO_OUT_REG) >> pin) & 1; }

// Nivel que realmente hay en el pad (registro de entrada).
// Solo es válido si el pin tiene la entrada habilitada (ver setup).
int nivelEnPin(int pin)   { return (REG_READ(GPIO_IN_REG) >> pin) & 1; }

// Detiene el programa hasta recibir cualquier carácter por el
// Monitor Serie. Antes descarta lo que haya quedado en el buffer.
void esperarUsuario()
{
  Serial.println("   -> envia cualquier caracter para continuar");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();
}

// Imprime lo escrito y lo leído en DIR y en el control MODE0,
// y avisa si lo escrito no coincide con lo que hay en el pad.
void reportar(int j)
{
  int dirEsc = nivelEscrito(DIR_PIN[j]);
  int dirPin = nivelEnPin(DIR_PIN[j]);
  int m0Esc  = nivelEscrito(MODE0_PIN[j]);
  int m0Pin  = nivelEnPin(MODE0_PIN[j]);

  Serial.printf("   DIR (GPIO %d): escrito=%d pin=%d | control MODE0 (GPIO %d): escrito=%d pin=%d\n",
                DIR_PIN[j], dirEsc, dirPin, MODE0_PIN[j], m0Esc, m0Pin);

  if (m0Esc != m0Pin)
    Serial.println("   !! El control MODE0 no coincide: la lectura del pad no es confiable");
  else if (dirEsc != dirPin)
    Serial.println("   !! DIR no coincide: algo externo fuerza el pin (corto o pad danado)");
}

// Genera n pulsos en STEP del motor j, a velocidad constante.
void pasos(int j, int n)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(STEP_PIN[j], HIGH);
    delayMicroseconds(PULSO_US);
    digitalWrite(STEP_PIN[j], LOW);
    delayMicroseconds(PERIODO_US - PULSO_US);
  }
}

// Prueba 1: deja DIR fijo en LOW y luego en HIGH, con el motor
// quieto, y espera en cada nivel para medir con el multímetro.
void pruebaEstatica(int j)
{
  Serial.printf("\n[Motor %d] Prueba estatica (motor quieto)\n", j + 1);

  for (int nivel : {LOW, HIGH}) {
    digitalWrite(DIR_PIN[j], nivel);
    delay(1);
    Serial.printf("  DIR en %s. Mide en el pin de la ESP32 y en el pin DIR del driver.\n",
                  nivel ? "HIGH (esperado ~3,3 V)" : "LOW (esperado ~0 V)");
    reportar(j);
    esperarUsuario();
  }
}

// Prueba 2: da PASOS pasos con DIR en LOW y luego en HIGH.
// El eje debería girar en un sentido y después volver.
void pruebaDinamica(int j)
{
  Serial.printf("\n[Motor %d] Prueba dinamica (observa el sentido de giro)\n", j + 1);

  for (int nivel : {LOW, HIGH}) {
    digitalWrite(DIR_PIN[j], nivel);
    delay(1);                        // DIR estable antes del primer paso
    Serial.printf("  %d pasos con DIR en %s\n", PASOS, nivel ? "HIGH" : "LOW");
    reportar(j);
    pasos(j, PASOS);
    delay(500);
  }
  Serial.println("  Si el eje NO volvio a su posicion inicial, el sentido no cambio.");
  esperarUsuario();
}

// ============================================================
// 4. SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  for (int j = 0; j < 2; j++) {
    // Todos los pines de control como salida
    for (int p : {STEP_PIN[j], DIR_PIN[j], EN_PIN[j], MODE0_PIN[j], MODE1_PIN[j]})
      pinMode(p, OUTPUT);

    // Habilita también la entrada en DIR y MODE0, para poder leer
    // el nivel real del pad mientras se usa como salida
    gpio_input_enable((gpio_num_t)DIR_PIN[j]);
    gpio_input_enable((gpio_num_t)MODE0_PIN[j]);

    // Estado inicial: driver deshabilitado, 1/8 de paso (MODE2 a GND)
    digitalWrite(EN_PIN[j], HIGH);
    digitalWrite(MODE0_PIN[j], HIGH);
    digitalWrite(MODE1_PIN[j], HIGH);
    digitalWrite(STEP_PIN[j], LOW);
    digitalWrite(DIR_PIN[j], LOW);
  }

  // Habilita las salidas de ambos drivers
  delay(10);
  for (int j = 0; j < 2; j++) digitalWrite(EN_PIN[j], LOW);

  Serial.println();
  Serial.println("===== Diagnostico DIR =====");
  Serial.println("Control: MODE0 debe leerse escrito=1 pin=1");
  esperarUsuario();
}

void loop()
{
  // Recorre ambos motores: primero la prueba estática, luego la dinámica
  for (int j = 0; j < 2; j++) {
    pruebaEstatica(j);
    pruebaDinamica(j);
  }

  Serial.println("\n===== Ciclo completo. Se repite. =====");
}

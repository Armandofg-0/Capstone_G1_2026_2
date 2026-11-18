// CÓDIGO PARA PROBAR EL MOVIMIENTO DE LOS MOTORES (Revisar el informe)
// * Datos:
// - Drivers: 2 x DRV8825 (motor 1 = Tilt, motor 2 = Pan)
// - Cada eje oscila entre -AMPLITUD_DEG y +AMPLITUD_DEG con rampas de aceleración. 
// - La posición al encender se toma como el centro (0°).

//  * Relaciones usadas (ver sección "STEP" del informe):
//   - k = 200 * N * R / 360   [pasos/°]
//   - f_step = k * omega [pasos/s]
//   - a_step = k * alpha [pasos/s^2]

#include <Arduino.h>
#include <AccelStepper.h>

// ============================================================
// 1. PARÁMETROS DE MOVIMIENTO (en el eje, después de la polea)
// ============================================================

const float AMPLITUD_DEG = 60.0;   // oscila entre -AMPLITUD y +AMPLITUD [°]
const float VEL_DEG_S    = 30.0;   // velocidad angular en estado estacionario [°/s]
const float ACEL_DEG_S2  = 60.0;   // aceleración alpha en cada rampa [°/s^2]

// Microstep por paso completo (N). Debe ser compatible con
// MODE2_CABLEADO LOW: 1, 2, 4 u 8.
const int MICROSTEPS = 8;

// Ancho del pulso STEP [us]. El DRV8825 exige >= 1,9 us.
const unsigned int STEP_PULSE_US = 3;

// ============================================================
// 2. PINES (ver tabla de conexiones del informe)
// ============================================================

// Motor 1 - Tilt
const int M1_DIR_PIN   = 25;
const int M1_STEP_PIN  = 26;
const int M1_EN_PIN    = 12;   // GPIO12 es pin de strapping del ESP32: no debe
                               // estar en HIGH al arrancar (nENBL tiene pull-down)
const int M1_MODE0_PIN = 14;
const int M1_MODE1_PIN = 27;

// Motor 2 - Pan
const int M2_DIR_PIN   = 16;
const int M2_STEP_PIN  = 17;
const int M2_EN_PIN    = 19;
const int M2_MODE0_PIN = 23;
const int M2_MODE1_PIN = 18;

// MODE2 va cableado en ambos drivers, no a un GPIO.
// LOW (a GND): hasta 1/8.  HIGH (a 3,3 V): 1/16 o 1/32.
const int MODE2_CABLEADO = LOW;

// SLEEP y RESET van cableados a 3,3 V en ambos drivers.

// ============================================================
// 3. MECÁNICA
// ============================================================

const float FULL_STEPS_PER_REV = 200.0;  // motor de 1,8° por paso

const float R_TILT = 4.0;   // polea 20 -> 80 dientes (motor 1)
const float R_PAN  = 3.2;   // polea 20 -> 64 dientes (motor 2)

// ============================================================
// 4. CONSTANTES DERIVADAS (NO EDITAR)
// ============================================================

// Pasos que hay que enviar por cada grado de giro del eje: k
const float K_TILT = FULL_STEPS_PER_REV * MICROSTEPS * R_TILT / 360.0;
const float K_PAN  = FULL_STEPS_PER_REV * MICROSTEPS * R_PAN  / 360.0;

// Posición de los extremos de la oscilación, en pasos (redondea a long int)
const long EXTREMO_TILT = lround(AMPLITUD_DEG * K_TILT);
const long EXTREMO_PAN  = lround(AMPLITUD_DEG * K_PAN);

// ============================================================
// 5. HANDLERS (objetos de la clase AccelStepper que manejan los drivers)
// ============================================================

// Modo DRIVER: la librería solo maneja los pines STEP y DIR.
AccelStepper motorTilt(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motorPan (AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// ============================================================
// 6. FUNCIONES
// ============================================================
// NivelesModo: traduce "micro" (que puede ser 1,2, 4, 8) a la configuración
// de los pines M0 y M1 para lograr dicha configuración de micro (si no es válido
// retorna "false").
bool nivelesModo(int micro, int &m0, int &m1)
{
  if (MODE2_CABLEADO == LOW) {
    switch (micro) {
      case 1: m0 = LOW;  m1 = LOW;  return true;
      case 2: m0 = HIGH; m1 = LOW;  return true;
      case 4: m0 = LOW;  m1 = HIGH; return true;
      case 8: m0 = HIGH; m1 = HIGH; return true;
    }
  }
  return false;
}
//----------------------------------------------------------------------------------------
// configurarDriver: Deja el driver deshabilitado (EN = HIGH) y fija los pines MODE0 y MODE1. 
// Las salidas se habilitan después, en setup().
// el flujo para cada pin lógico es: 
// habilitar la salida (pinMode) -> asignarle el nivel lógico deseado (digitalWrite)
void configurarDriver(int en, int mode0, int mode1, int m0, int m1)
{
  pinMode(en, OUTPUT);
  digitalWrite(en, HIGH); // salidas apagadas mientras se fija el modo
  pinMode(mode0, OUTPUT);
  pinMode(mode1, OUTPUT);
  digitalWrite(mode0, m0);
  digitalWrite(mode1, m1);
}
//----------------------------------------------------------------------------------------
// configurarMotor: Convierte velocidad y aceleración del eje a pasos con k
// (f_step = k*omega, a_step = k*alpha), toma la posición actual como 0 y fija el primer objetivo de distancia.
void configurarMotor(AccelStepper &m, float k, long extremo)
{
  m.setMinPulseWidth(STEP_PULSE_US);
  m.setMaxSpeed(VEL_DEG_S * k);      // [pasos/s]
  m.setAcceleration(ACEL_DEG_S2 * k); // [pasos/s^2]
  m.setCurrentPosition(0); // la pose al encender es el centro
  m.moveTo(extremo);
}
//----------------------------------------------------------------------------------------
// oscilar: Al llegar a un extremo, cambia el objetivo al extremo opuesto.
// run() emite como máximo un step por llamada, así que esta función
// debe llamarse lo más seguido posible (desde loop(), sin delay()).
void oscilar(AccelStepper &m, long extremo)
{
  if (m.distanceToGo() == 0) // se llegó al setpoint de distancia
  // distancia positiva -> cambio a extremo negativo ("?")
  // distancia negativa -> cambio a extremo positivo (":")
    m.moveTo(m.currentPosition() > 0 ? -extremo : extremo);
  m.run(); // internamente maneja las condiciones para generar el step
}

// ============================================================
// 7. SETUP / LOOP
// ============================================================

void setup()
{
  // Para debug: Inicializa monitor serial y da tiempo para ello.
  Serial.begin(115200);
  delay(500);

  // Niveles MODE0/MODE1 para el microstepping elegido
  int m0, m1;
  if (!nivelesModo(MICROSTEPS, m0, m1)) {
    Serial.println("MICROSTEPS no es valido para el nivel de MODE2 cableado");
    while (true) delay(1000); // se detiene el programa: configuración inválida
  }

  // Drivers: deshabilitados y con el modo fijado
  configurarDriver(M1_EN_PIN, M1_MODE0_PIN, M1_MODE1_PIN, m0, m1);
  configurarDriver(M2_EN_PIN, M2_MODE0_PIN, M2_MODE1_PIN, m0, m1);

  // Perfil de movimiento de cada eje
  configurarMotor(motorTilt, K_TILT, EXTREMO_TILT);
  configurarMotor(motorPan,  K_PAN,  EXTREMO_PAN);

  // Habilita las salidas
  delay(10);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);

  Serial.printf("Oscilacion +-%.0f deg a %.0f deg/s\n", AMPLITUD_DEG, VEL_DEG_S);
  Serial.printf("Tilt: k = %.2f pasos/deg, extremo = %ld pasos\n", K_TILT, EXTREMO_TILT);
  Serial.printf("Pan:  k = %.2f pasos/deg, extremo = %ld pasos\n", K_PAN,  EXTREMO_PAN);
}

void loop()
{
  oscilar(motorTilt, EXTREMO_TILT);
  oscilar(motorPan,  EXTREMO_PAN);
}

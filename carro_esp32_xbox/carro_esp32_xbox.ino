/*
  =====================================================================
  CONTROL DE CARRITO RC CON ESP32 + CONTROL DE XBOX (BLUEPAD32)
  =====================================================================

  Este código permite controlar un carrito RC con dirección por
  inversión de giro (tipo tanque/diferencial) usando un control de
  Xbox conectado por Bluetooth directamente al ESP32, mediante la
  librería Bluepad32.

  Esquema de control:
    - RT (throttle): avanza recto. | LT (brake): retrocede recto.
    - RB (R1): giro tank a la derecha. | LB (L1): giro tank a la izquierda.
      Los gatillos tienen prioridad sobre el giro.
    - Joystick izquierdo Y: sube/baja la pala (servo).
    - X: amplificador (toggle). Y/B: intermitentes.

  -------------------------------------------------------------------
  ADVERTENCIA DE SEGURIDAD - LEER ANTES DE MODIFICAR ESTE CÓDIGO
  -------------------------------------------------------------------
  Este carrito usa un puente H para cada lado (izquierdo y derecho).
  Si se activan AL MISMO TIEMPO la salida de "adelante" y la de
  "reversa" de un mismo lado, se produce un CORTOCIRCUITO dentro del
  puente H, lo cual puede DESTRUIR el puente H y los optocopladores
  (e incluso representar un riesgo de incendio/explosión de los
  componentes de potencia).

  Por esta razón, la función actualizarLado() es la ÚNICA parte del
  código autorizada a escribir físicamente (digitalWrite) sobre los
  pines de los puentes H (DW1, DW2, DW3 y DW4). Ninguna otra función
  debe tocar esos pines directamente, sin importar qué tan "segura"
  parezca la lógica que la generó. Esta regla no es configurable y no
  debe eliminarse ni "optimizarse".
  =====================================================================
*/

#include <Bluepad32.h>
#include <esp_system.h> // para esp_reset_reason() (diagnóstico de reinicios)
#include "soc/soc.h"         // para WRITE_PERI_REG
#include "soc/rtc_cntl_reg.h" // para RTC_CNTL_BROWN_OUT_REG
#include "gap.h"             // BTstack: gap_set_local_name() para el nombre BT del ESP32

// =====================================================================
// SECCIÓN 1: CONFIGURACIÓN - PINES DE SALIDA
// =====================================================================
#define DW1  14   // Puente H - Izquierdo, ADELANTE
#define DW2  27   // Puente H - Izquierdo, REVERSA
#define DW3  26   // Puente H - Derecho, REVERSA
#define DW4  25   // Puente H - Derecho, ADELANTE
#define DW5  33   // Amplificador de audio
#define DW6  32   // Sensor / luz de proximidad de reversa
#define DW7  15   // Intermitente / direccional 1
#define DW8  4    // Intermitente / direccional 2
#define PIN_SERVO 13  // Señal PWM del servo (GPIO35 no sirve: es solo entrada)

// =====================================================================
// SECCIÓN 1B: CONFIGURACIÓN - SERVO DE LA PALA (EJE Y DEL JOYSTICK IZQ.)
// =====================================================================
// El servo ya no se usa para dirección: ahora levanta/baja la pala
// usando el eje Y del joystick izquierdo (el eje X se usa para girar).
// Ángulos límite del servo. Ajusta según el rango físico real que
// tolera tu mecanismo para no forzarlo.
#define SERVO_ANGULO_MIN     0
#define SERVO_ANGULO_MAX     180
#define SERVO_ANGULO_CENTRO  90

// Zona muerta del joystick: valores del eje Y por debajo de este
// umbral (en valor absoluto) se ignoran y la pala se queda quieta en
// su posición actual, para evitar temblor por el "ruido" natural del
// joystick cuando está soltado.
#define SERVO_DEADZONE        15

// Rango típico del eje Y en Bluepad32: aproximadamente -512 a 512
// (puede variar un poco según el control). Si el servo no llega a
// los extremos, ajusta estos valores tras probar con la consola.
#define JOYSTICK_Y_MIN      -512
#define JOYSTICK_Y_MAX       512

// Configuración del PWM nativo del ESP32 (reemplaza a la librería
// ESP32Servo, que resultó incompatible con este core/placa).
// NOTA: el core "esp32-bluepad32" usa la API ANTIGUA de LEDC
// (ledcSetup + ledcAttachPin + ledcWrite, con número de canal),
// no la API nueva de un solo pin (ledcAttach) de cores más recientes.
#define SERVO_PWM_FREQ_HZ      50   // frecuencia estándar de un servo
#define SERVO_PWM_RESOLUTION   12   // bits del duty cycle (4095 pasos: ~0.46us/paso, más que suficiente; antes 16 bits)
#define SERVO_PULSO_MIN_US     500  // ancho de pulso en el ángulo mínimo
#define SERVO_PULSO_MAX_US     2400 // ancho de pulso en el ángulo máximo
#define SERVO_LEDC_CHANNEL     4    // canal LEDC dedicado al servo (0-15, no usado por nada más)

// Prueba de arranque: al encender, el servo hace un barrido
// 90 -> 0 -> 90 -> 180 -> 90. Sirve para confirmar que el servo y el
// PWM del ESP32 funcionan, sin depender del control. Ponlo en false
// cuando termines de diagnosticar.
#define SERVO_TEST_ON_BOOT      true

// =====================================================================
// SECCIÓN 2: CONFIGURACIÓN - MAPEO DE BOTONES (FÁCIL DE CAMBIAR)
// =====================================================================
// Umbral para considerar presionados los triggers analógicos (0-1023
// en Bluepad32, según el controller). Ajusta si tu control no llega a
// fondo o se activa demasiado sensible.
#define TRIGGER_THRESHOLD        50

// Retardo de seguridad (ms) que se exige con la salida TOTALMENTE
// APAGADA al cambiar de sentido, antes de encender la salida contraria
// del mismo lado. A diferencia de la versión anterior, este retardo se
// aplica ante CUALQUIER cambio de dirección, incluso si el usuario
// suelta y vuelve a presionar rápido.
#define DIRECTION_CHANGE_DELAY_MS 150

// Modo consola de depuración: true = imprime por Serial, false = apagado
#define DEBUG_MODE                true

// Cada cuántos ms se imprime en vivo el estado crudo del control
// (LT/RT/LB/RB/joysticks) cuando DEBUG_MODE está activo. Esto sirve
// para verificar en el Monitor Serial (115200 baud) que el ESP32 está
// recibiendo datos del control.
#define DEBUG_INTERVAL_MS         300

// Intervalo de parpadeo de los intermitentes (ms)
#define BLINK_INTERVAL_MS         400

// Timeout de datos del control: si pasan estos ms SIN que llegue un
// paquete nuevo del control, se asume desconexión y se aplica el
// failsafe aunque isConnected() aún reporte enlace.
// IMPORTANTE: 0 = DESACTIVADO. Muchos controles (especialmente clones
// o gamepads genéricos) solo envían datos cuando algo CAMBIA, no de
// forma periódica, y un timeout corto cortaría el control de forma
// falsa. Con 0, el failsafe depende de isConnected()/desconexión de
// Bluepad32, que es confiable. Actívalo solo si sabes que tu control
// reporta en intervalos fijos.
#define TIMEOUT_CONTROL_MS        0

// Si es true, se borran las claves de emparejamiento en cada boot
// (fuerza un re-emparejamiento limpio, pero el control debe volver a
// vincularse cada vez que enciendes el carrito).
// IMPORTANTE: déjalo en FALSE. Si tu alimentación por pines hace que
// el ESP32 se reinicie al quitar el USB, con TRUE cada reinicio borra
// las claves y el control pierde la vinculación. Con FALSE, el ESP32
// recuerda el emparejamiento y el control se reconecta solo tras un
// reinicio.
#define FORGET_KEYS_ON_BOOT       false

// --- Botones del gamepad usados para funciones auxiliares ---
// Reasigna estos IDs sin tocar el resto del código. La lista completa
// de botones está en la enumeración BTN_ID_* (sección 4B).
//
// MAPEO DE CONTROL:
//   RT (throttle)         -> marcha ADELANTE (ambos lados)
//   LT (brake)            -> marcha REVERSA (ambos lados)
//   RB (R1)               -> giro tank a la DERECHA (izq adelante + der reversa)
//   LB (L1)               -> giro tank a la IZQUIERDA (izq reversa + der adelante)
//   Joystick izquierdo Y  -> sube/baja la pala (servo)
//   X                     -> amplificador (toggle)
//   Y / B                 -> intermitentes 1 y 2
// Los gatillos tienen prioridad sobre el giro: si RT o LT está
// presionado, LB/RB no giran.
#define BTN_AMPLIFICADOR     BTN_ID_X
#define BTN_INTERMITENTE_1   BTN_ID_Y
#define BTN_INTERMITENTE_2   BTN_ID_B
#define BTN_GIRO_IZQ         BTN_ID_L1
#define BTN_GIRO_DER         BTN_ID_R1

// =====================================================================
// SECCIÓN 2B: CONFIGURACIÓN - MONITOREO DE BATERÍA (OPCIONAL)
// =====================================================================
// Requiere un divisor de voltaje conectado al pin de ADC indicado
// (escala el voltaje de la batería para que quede por debajo de 3.3V).
// Con MONITOR_BATERIA_ENABLED false este bloque queda completamente
// inactivo y no afecta el funcionamiento.
#define MONITOR_BATERIA_ENABLED  false
#define PIN_BATERIA_ADC          35   // pin de solo-entrada, libre en esta placa
#define VOLTAJE_DIVISOR          2.0f // factor del divisor: (R1+R2)/R2
#define VOLTAJE_CORTE_V          6.0f // voltaje mínimo de batería (LiPo 2S ~6.4V)
#define REVISION_BATERIA_MS      500  // cada cuánto se mide (ms)

// =====================================================================
// SECCIÓN 3: VARIABLES DE ESTADO
// =====================================================================
ControllerPtr myController = nullptr;
int anguloServoActual = SERVO_ANGULO_CENTRO;

// Salidas "deseadas" según los botones (antes del interlock de seguridad)
bool deseaIzqAdelante = false;
bool deseaIzqReversa  = false;
bool deseaDerAdelante = false;
bool deseaDerReversa  = false;

// Estado REAL actual de las salidas físicas (después del interlock).
// El estado de los pines del puente H (DW1-DW4) vive dentro de la
// estructura LadoPuenteH (sección 3B).
bool estadoDW5 = false; // amplificador
bool estadoDW6 = false; // sensor reversa
bool estadoDW7 = false; // intermitente 1
bool estadoDW8 = false; // intermitente 2

// Control de parpadeo de intermitentes
unsigned long ultimoParpadeo = 0;
bool estadoParpadeo = false;

// Estado de conexión y datos del control
bool controlConectado = false;
unsigned long ultimoDatoRecibido = 0;

// Monitoreo de batería
unsigned long ultimaRevisionBateria = 0;
bool bateriaBaja = false;

// =====================================================================
// SECCIÓN 3B: ESTRUCTURA DE UN LADO DEL PUENTE H
// =====================================================================
// Encapsula todo el estado de uno de los dos lados (izquierdo o
// derecho). Permite que la lógica de interlock esté escrita UNA sola
// vez y se reutilice para ambos lados, eliminando el código duplicado.
struct LadoPuenteH {
  uint8_t pinAdelante;    // pin de la salida "adelante" de este lado
  uint8_t pinReversa;     // pin de la salida "reversa" de este lado
  bool estadoAdelante;    // estado REAL de la salida adelante
  bool estadoReversa;     // estado REAL de la salida reversa
  bool apagadoTotalPrev;  // el lado estuvo totalmente apagado en el ciclo anterior
  unsigned long momentoApagado; // instante en que el lado quedó totalmente apagado
};

// Inicialmente apagadoTotalPrev = true para que el primer comando no
// se retrase por el interlock al arrancar (momentoApagado queda en 0).
LadoPuenteH ladoIzq = { DW1, DW2, false, false, true, 0 };
LadoPuenteH ladoDer = { DW4, DW3, false, false, true, 0 };

// =====================================================================
// SECCIÓN 4: CALLBACKS DE CONEXIÓN DE BLUEPAD32
// =====================================================================
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    myController = ctl;
    ultimoDatoRecibido = millis();
    Serial.println(F("[BT] Control de Xbox CONECTADO."));
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    myController = nullptr;
    Serial.println(F("[BT] Control de Xbox DESCONECTADO. Aplicando failsafe."));
  }
}

// =====================================================================
// SECCIÓN 4B: LECTURA DE BOTONES POR ID
// =====================================================================
// Reemplaza a las macros frágiles del tipo "#define BTN_X ctl->x()"
// (que dependían de que la variable se llamara "ctl"). Ahora el mapeo
// se configura solo con los BTN_ID_* de la sección 2.
enum {
  BTN_ID_A,
  BTN_ID_B,
  BTN_ID_X,
  BTN_ID_Y,
  BTN_ID_L1,
  BTN_ID_R1,
  BTN_ID_L2,
  BTN_ID_R2,
  BTN_ID_THUMBL,
  BTN_ID_THUMBR,
  BTN_ID_START,
  BTN_ID_SELECT,
  BTN_ID_SYSTEM,
  BTN_ID_DPAD_UP,
  BTN_ID_DPAD_DOWN,
  BTN_ID_DPAD_LEFT,
  BTN_ID_DPAD_RIGHT,
  BTN_ID_NINGUNO
};

bool leerBoton(ControllerPtr ctl, int botonId) {
  if (ctl == nullptr) return false;
  switch (botonId) {
    case BTN_ID_A:        return ctl->a();
    case BTN_ID_B:        return ctl->b();
    case BTN_ID_X:        return ctl->x();
    case BTN_ID_Y:        return ctl->y();
    case BTN_ID_L1:       return ctl->l1();
    case BTN_ID_R1:       return ctl->r1();
    case BTN_ID_L2:       return ctl->l2();
    case BTN_ID_R2:       return ctl->r2();
    case BTN_ID_THUMBL:   return ctl->thumbL();
    case BTN_ID_THUMBR:   return ctl->thumbR();

    default: return false;
  }
}

// =====================================================================
// SECCIÓN 5: LECTURA DEL CONTROL -> CÁLCULO DE SALIDAS DESEADAS
// =====================================================================
// Traduce el estado del control en lo que el usuario QUIERE que pase.
// Prioridades (de mayor a menor): RT/LT (gatillos) -> LB/RB (giro tank).
// Esta función NO toca ningún pin físico.
void calcularSalidasDeseadas(ControllerPtr ctl) {
  // Triggers analógicos: throttle() = RT, brake() = LT en Bluepad32.
  int valorLT = ctl->brake();     // Left Trigger
  int valorRT = ctl->throttle();  // Right Trigger

  bool ltPresionado = valorLT > TRIGGER_THRESHOLD;
  bool rtPresionado = valorRT > TRIGGER_THRESHOLD;

  bool giroIzq = leerBoton(ctl, BTN_GIRO_IZQ); // LB -> giro a la izquierda
  bool giroDer = leerBoton(ctl, BTN_GIRO_DER); // RB -> giro a la derecha

  // Reiniciar deseos en cada ciclo
  deseaIzqAdelante = false;
  deseaIzqReversa  = false;
  deseaDerAdelante = false;
  deseaDerReversa  = false;

  if (rtPresionado) {
    // RT tiene prioridad absoluta: adelante en ambos lados.
    deseaIzqAdelante = true;
    deseaDerAdelante = true;
  } else if (ltPresionado) {
    // LT tiene prioridad absoluta: reversa en ambos lados.
    deseaIzqReversa = true;
    deseaDerReversa = true;
  } else if (giroIzq && giroDer) {
    // Ambos botones a la vez: sin orden -> todo apagado (marcha neutra).
  } else if (giroDer) {
    // RB -> giro tank a la DERECHA: el lado izquierdo avanza y el
    // derecho retrocede (gira en el sitio).
    deseaIzqAdelante = true;
    deseaDerReversa  = true;
  } else if (giroIzq) {
    // LB -> giro tank a la IZQUIERDA: el lado izquierdo retrocede y
    // el derecho avanza.
    deseaIzqReversa = true;
    deseaDerAdelante = true;
  }
  // Sin gatillos ni botones de giro -> todo apagado.
}

// =====================================================================
// SECCIÓN 6: INTERLOCK DE SEGURIDAD (ÚNICO CÓDIGO QUE TOCA DW1-DW4)
// =====================================================================
// Recibe lo que el usuario "desea" y filtra esas órdenes para
// garantizar que adelante y reversa de un mismo lado NUNCA coincidan
// encendidas, exigiendo además un mínimo de DIRECTION_CHANGE_DELAY_MS
// con la salida totalmente apagada ante CUALQUIER cambio de dirección
// (no solo cuando se cambia directo de adelante a reversa).
void actualizarLado(LadoPuenteH &lado, bool deseaAdelante, bool deseaReversa, unsigned long ahora) {
  bool totalmenteApagado = !lado.estadoAdelante && !lado.estadoReversa;

  // Registrar el instante en que este lado quedó por completo apagado
  // (flanco de bajada de "totalmente apagado").
  if (totalmenteApagado && !lado.apagadoTotalPrev) {
    lado.momentoApagado = ahora;
  }
  lado.apagadoTotalPrev = totalmenteApagado;

  bool quiereAdelante = deseaAdelante && !deseaReversa;
  bool quiereReversa  = deseaReversa  && !deseaAdelante;

  if (quiereAdelante) {
    if (lado.estadoAdelante) {
      // Ya está en adelante: nada que hacer.
    } else if (totalmenteApagado &&
               (ahora - lado.momentoApagado >= DIRECTION_CHANGE_DELAY_MS)) {
      // Lleva el tiempo mínimo apagado exigido: encender adelante.
      lado.estadoAdelante = true;
      lado.estadoReversa  = false;
    } else if (!totalmenteApagado) {
      // Estaba en reversa y se pide adelante: primero apagar y esperar.
      lado.estadoAdelante = false;
      lado.estadoReversa  = false;
    }
    // Si está apagado pero aún no cumple el retardo: esperar.
  } else if (quiereReversa) {
    if (lado.estadoReversa) {
      // Ya está en reversa: nada que hacer.
    } else if (totalmenteApagado &&
               (ahora - lado.momentoApagado >= DIRECTION_CHANGE_DELAY_MS)) {
      lado.estadoAdelante = false;
      lado.estadoReversa  = true;
    } else if (!totalmenteApagado) {
      lado.estadoAdelante = false;
      lado.estadoReversa  = false;
    }
  } else {
    // Sin orden (o, por algún bug, ambas a la vez): apagar ambas
    // salidas. Nunca se permite que ambas queden encendidas.
    lado.estadoAdelante = false;
    lado.estadoReversa  = false;
  }

  // Barrera final absoluta: bajo NINGUNA circunstancia deben quedar
  // ambas encendidas a la vez, sin importar el resto de la lógica.
  if (lado.estadoAdelante && lado.estadoReversa) {
    lado.estadoAdelante = false;
    lado.estadoReversa  = false;
  }

  digitalWrite(lado.pinAdelante, lado.estadoAdelante ? HIGH : LOW);
  digitalWrite(lado.pinReversa,  lado.estadoReversa  ? HIGH : LOW);
}

void aplicarInterlockSeguridad() {
  unsigned long ahora = millis();
  actualizarLado(ladoIzq, deseaIzqAdelante, deseaIzqReversa, ahora);
  actualizarLado(ladoDer, deseaDerAdelante, deseaDerReversa, ahora);

  // Sensor/luz de reversa: se activa si cualquiera de los dos lados
  // está en reversa.
  estadoDW6 = ladoIzq.estadoReversa || ladoDer.estadoReversa;
  digitalWrite(DW6, estadoDW6 ? HIGH : LOW);
}

// =====================================================================
// SECCIÓN 7: SALIDAS AUXILIARES (AMPLIFICADOR E INTERMITENTES)
// =====================================================================
void actualizarSalidasAuxiliares(ControllerPtr ctl) {
  // Amplificador: funciona como interruptor (toggle). Cada vez que se
  // PRESIONA el botón (flanco de subida, no mientras se mantiene), el
  // estado de DW5 se invierte.
  static bool botonAmplifPrevio = false;
  bool botonAmplifActual = leerBoton(ctl, BTN_AMPLIFICADOR);

  if (botonAmplifActual && !botonAmplifPrevio) {
    estadoDW5 = !estadoDW5;
  }
  botonAmplifPrevio = botonAmplifActual;

  digitalWrite(DW5, estadoDW5 ? HIGH : LOW);

  // Parpadeo compartido para ambos intermitentes
  unsigned long ahora = millis();
  if (ahora - ultimoParpadeo >= BLINK_INTERVAL_MS) {
    ultimoParpadeo = ahora;
    estadoParpadeo = !estadoParpadeo;
  }

  bool intermitente1Activo = leerBoton(ctl, BTN_INTERMITENTE_1);
  bool intermitente2Activo = leerBoton(ctl, BTN_INTERMITENTE_2);

  estadoDW7 = intermitente1Activo && estadoParpadeo;
  estadoDW8 = intermitente2Activo && estadoParpadeo;

  digitalWrite(DW7, estadoDW7 ? HIGH : LOW);
  digitalWrite(DW8, estadoDW8 ? HIGH : LOW);
}

// =====================================================================
// SECCIÓN 7B: CONTROL DEL SERVO DE LA PALA (EJE Y DEL JOYSTICK IZQ.)
// =====================================================================
// Convierte un ángulo (0-180) al duty cycle correspondiente y lo
// escribe directamente con la API nativa de PWM del ESP32 (ledcWrite),
// sin depender de la librería ESP32Servo. El servo levanta/baja la
// pala según el eje Y del joystick izquierdo.
void escribirAnguloServo(int angulo) {
  angulo = constrain(angulo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);

  long pulsoUs = map(angulo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX,
                      SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);

  unsigned long periodoUs = 1000000UL / SERVO_PWM_FREQ_HZ; // 20000us a 50Hz
  unsigned long dutyMax = (1UL << SERVO_PWM_RESOLUTION) - 1;

  uint32_t duty = (uint32_t)(((unsigned long)pulsoUs * dutyMax) / periodoUs);

  ledcWrite(SERVO_LEDC_CHANNEL, duty);
}

void actualizarServo(ControllerPtr ctl) {
  int ejeY = ctl->axisY(); // eje Y del joystick IZQUIERDO en Bluepad32

  // Zona muerta: si el joystick está cerca del centro, no tocamos el
  // servo, para que la pala no tiemble sola por el ruido del sensor.
  if (abs(ejeY) < SERVO_DEADZONE) {
    return; // la pala se queda en su ángulo actual
  }

  // Mapeamos el eje Y (rango del joystick) al rango de ángulos del
  // servo. En Bluepad32, Y negativo suele ser "arriba" y positivo
  // "abajo" -> por eso invertimos el mapeo para que arriba del
  // joystick levante la pala.
  int angulo = map(ejeY, JOYSTICK_Y_MIN, JOYSTICK_Y_MAX,
                    SERVO_ANGULO_MAX, SERVO_ANGULO_MIN);
  angulo = constrain(angulo, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);

  if (angulo != anguloServoActual) {
    anguloServoActual = angulo;
    escribirAnguloServo(anguloServoActual);
  }
}

// =====================================================================
// SECCIÓN 7C: MONITOREO DE BATERÍA (OPCIONAL)
// =====================================================================
void revisarBateria() {
  if (!MONITOR_BATERIA_ENABLED) return;

  unsigned long ahora = millis();
  if (ahora - ultimaRevisionBateria < REVISION_BATERIA_MS) return;
  ultimaRevisionBateria = ahora;

  // Asume lectura ADC de 12 bits con atenuación de 11dB (rango 0-3.3V).
  float voltaje = (analogRead(PIN_BATERIA_ADC) / 4095.0f) * 3.3f * VOLTAJE_DIVISOR;
  bateriaBaja = (voltaje < VOLTAJE_CORTE_V);

  if (DEBUG_MODE) {
    Serial.print(F("[BATERIA] Voltaje: "));
    Serial.print(voltaje);
    Serial.println(bateriaBaja ? F(" V (BAJA)") : F(" V"));
  }
}

// =====================================================================
// SECCIÓN 8: FAILSAFE - APAGA TODO SI SE PIERDE EL CONTROL
// =====================================================================
void aplicarFailsafe() {
  deseaIzqAdelante = false;
  deseaIzqReversa  = false;
  deseaDerAdelante = false;
  deseaDerReversa  = false;

  // Se sigue pasando por el interlock para mantener consistente el
  // estado de las banderas de transición y apagar los puentes H.
  aplicarInterlockSeguridad();

  estadoDW5 = false;
  estadoDW7 = false;
  estadoDW8 = false;
  digitalWrite(DW5, LOW);
  digitalWrite(DW7, LOW);
  digitalWrite(DW8, LOW);
}

// =====================================================================
// SECCIÓN 9: CONSOLA DE DEPURACIÓN
// =====================================================================
void debugConsola(ControllerPtr ctl) {
  if (!DEBUG_MODE) return;

  // Solo imprime cuando el estado real de alguna salida física cambia,
  // o cuando cambia el estado de conexión del control. Así la consola
  // no se satura repitiendo el mismo bloque una y otra vez mientras
  // nada se mueve.
  static bool prevDW1 = false, prevDW2 = false, prevDW3 = false, prevDW4 = false;
  static bool prevDW5 = false, prevDW6 = false, prevDW7 = false, prevDW8 = false;
  static bool prevConectado = false;
  static bool prevBateriaBaja = false;
  static int prevAngulo = -1;
  static bool primeraVez = true;

  // Volcado periódico de datos crudos del control: se imprime en vivo
  // cada DEBUG_INTERVAL_MS aunque nada cambie (a diferencia del bloque
  // de "cambio" de abajo), para verificar que el ESP32 recibe datos.
  static unsigned long ultimoDumpDatos = 0;
  if (ctl != nullptr && controlConectado) {
    unsigned long ahora = millis();
    if (ahora - ultimoDumpDatos >= DEBUG_INTERVAL_MS) {
      ultimoDumpDatos = ahora;
      Serial.print(F("[DATA] LT=")); Serial.print(ctl->brake());
      Serial.print(F(" RT="));       Serial.print(ctl->throttle());
      Serial.print(F(" LB="));       Serial.print(ctl->l1());
      Serial.print(F(" RB="));       Serial.print(ctl->r1());
      Serial.print(F(" JoyX="));     Serial.print(ctl->axisX());
      Serial.print(F(" JoyY="));     Serial.println(ctl->axisY());
    }
  }

  bool dw1 = ladoIzq.estadoAdelante;
  bool dw2 = ladoIzq.estadoReversa;
  bool dw3 = ladoDer.estadoReversa;
  bool dw4 = ladoDer.estadoAdelante;

  bool cambio = primeraVez ||
                (dw1 != prevDW1) || (dw2 != prevDW2) ||
                (dw3 != prevDW3) || (dw4 != prevDW4) ||
                (estadoDW5 != prevDW5) || (estadoDW6 != prevDW6) ||
                (estadoDW7 != prevDW7) || (estadoDW8 != prevDW8) ||
                (anguloServoActual != prevAngulo) ||
                (controlConectado != prevConectado) ||
                (bateriaBaja != prevBateriaBaja);

  if (!cambio) return;

  primeraVez = false;
  prevDW1 = dw1; prevDW2 = dw2; prevDW3 = dw3; prevDW4 = dw4;
  prevDW5 = estadoDW5; prevDW6 = estadoDW6; prevDW7 = estadoDW7; prevDW8 = estadoDW8;
  prevAngulo = anguloServoActual;
  prevConectado = controlConectado;
  prevBateriaBaja = bateriaBaja;

  Serial.println(F("---------------------------------------------------"));
  Serial.print(F("[CONTROL] Conectado: "));
  Serial.println(controlConectado ? "SI" : "NO");

  if (ctl != nullptr && controlConectado) {
    Serial.print(F("  LT (brake): "));    Serial.print(ctl->brake());
    Serial.print(F("   RT (throttle): ")); Serial.println(ctl->throttle());
    Serial.print(F("  LB (giro izq): ")); Serial.print(ctl->l1());
    Serial.print(F("   RB (giro der): ")); Serial.println(ctl->r1());
    Serial.print(F("  Joy X: ")); Serial.print(ctl->axisX());
    Serial.print(F("   Joy Y (pala): ")); Serial.println(ctl->axisY());
  }

  Serial.println(F("[SALIDAS FISICAS]"));
  Serial.print(F("  DW1 (izq adelante): ")); Serial.println(dw1 ? "ON" : "OFF");
  Serial.print(F("  DW2 (izq reversa) : ")); Serial.println(dw2 ? "ON" : "OFF");
  Serial.print(F("  DW3 (der reversa) : ")); Serial.println(dw3 ? "ON" : "OFF");
  Serial.print(F("  DW4 (der adelante): ")); Serial.println(dw4 ? "ON" : "OFF");
  Serial.print(F("  DW5 (amplificador): ")); Serial.println(estadoDW5 ? "ON" : "OFF");
  Serial.print(F("  DW6 (sensor rev)  : ")); Serial.println(estadoDW6 ? "ON" : "OFF");
  Serial.print(F("  DW7 (intermit. 1) : ")); Serial.println(estadoDW7 ? "ON" : "OFF");
  Serial.print(F("  DW8 (intermit. 2) : ")); Serial.println(estadoDW8 ? "ON" : "OFF");
  Serial.print(F("  Pala (angulo)    : ")); Serial.println(anguloServoActual);

  if (MONITOR_BATERIA_ENABLED) {
    Serial.print(F("[BATERIA] Estado: "));
    Serial.println(bateriaBaja ? "BAJA (failsafe activo)" : "OK");
  }
}

// =====================================================================
// SECCIÓN 10: SETUP Y LOOP
// =====================================================================
// Traduce el motivo del último reinicio del ESP32 a texto, para saber
// si el chip se está reiniciando solo (energía/crash) o si el control
// es el que se desconecta.
const char* nombreRazonReset(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:     return "desconocida";
    case ESP_RST_POWERON:     return "encendido / perdida TOTAL de alimentacion";
    case ESP_RST_EXT:         return "reinicio externo (boton EN)";
    case ESP_RST_SW:          return "reinicio por software";
    case ESP_RST_PANIC:       return "PANICO / crash del firmware";
    case ESP_RST_INT_WDT:     return "watchdog de interrupcion";
    case ESP_RST_TASK_WDT:    return "watchdog de tarea";
    case ESP_RST_WDT:         return "watchdog";
    case ESP_RST_DEEPSLEEP:   return "deep sleep";
    case ESP_RST_BROWNOUT:    return "BROWNOUT (bajo voltaje de alimentacion)";
    case ESP_RST_SDIO:        return "SDIO";
    default:                  return "otra";
  }
}

void setup() {
  // Deshabilitar el detector de brownout a nivel de registro (este core
  // no expone brownout_disable()). Evita que una caída breve de voltaje
  // (p. ej. al conmutar de USB a batería, o al arrancar los motores)
  // reinicie el ESP32 y corte el Bluetooth.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(500);
  Serial.println(F("Iniciando sistema de control del carrito..."));
  Serial.print(F("[RST] Ultima razon de reset: "));
  Serial.println(nombreRazonReset(esp_reset_reason()));
  Serial.println(F("[PWR] Brownout detector DESACTIVADO (cambios de fuente de alimentación no reinician el ESP32)."));

  // Configurar todos los pines de salida y dejarlos apagados
  int pines[] = {DW1, DW2, DW3, DW4, DW5, DW6, DW7, DW8};
  for (int i = 0; i < 8; i++) {
    pinMode(pines[i], OUTPUT);
    digitalWrite(pines[i], LOW);
  }

  // Inicializar servo con PWM nativo del ESP32 (API antigua de LEDC:
  // ledcSetup + ledcAttachPin), sin usar la librería ESP32Servo, que
  // resultó incompatible con este core.
  double freqReal = ledcSetup(SERVO_LEDC_CHANNEL, SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION);
  if (freqReal == 0) {
    Serial.println(F("[SERVO] ERROR: ledcSetup() falló. Revisa SERVO_LEDC_CHANNEL y SERVO_PWM_RESOLUTION."));
  } else {
    ledcAttachPin(PIN_SERVO, SERVO_LEDC_CHANNEL);
    Serial.println(F("[SERVO] ledcSetup()/ledcAttachPin() OK."));
  }
  escribirAnguloServo(SERVO_ANGULO_CENTRO);
  Serial.println(F("[SERVO] Se mandó comando al centro (90 grados)."));

  // Prueba de barrido al arrancar: si el servo NO se mueve aquí, el
  // problema es de hardware/PWM y no del control Bluetooth.
  if (SERVO_TEST_ON_BOOT) {
    Serial.println(F("[SERVO] Test de barrido en boot: 90 -> 0 -> 90 -> 180 -> 90"));
    escribirAnguloServo(SERVO_ANGULO_MIN);
    delay(400);
    escribirAnguloServo(SERVO_ANGULO_CENTRO);
    delay(400);
    escribirAnguloServo(SERVO_ANGULO_MAX);
    delay(400);
    escribirAnguloServo(SERVO_ANGULO_CENTRO);
    delay(400);
    Serial.println(F("[SERVO] Test de barrido terminado."));
  }

  if (MONITOR_BATERIA_ENABLED) {
    analogSetPinAttenuation(PIN_BATERIA_ADC, ADC_11db);
    Serial.println(F("[BATERIA] Monitoreo activo."));
  }

  // Inicializar Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Nombre Bluetooth del ESP32 (aparece en los escaneos BT).
  // Se llama DESPUÉS de BP32.setup() para que el nombre se aplique al
  // stack ya inicializado.
  gap_set_local_name("patroclo");
  Serial.println(F("[BT] Nombre del dispositivo: patroclo"));

  if (FORGET_KEYS_ON_BOOT) {
    BP32.forgetBluetoothKeys(); // fuerza nuevo emparejamiento limpio
  }

  Serial.println(F("Sistema listo. Esperando control de Xbox..."));
}

void loop() {
  // BP32.update() actualiza internamente el estado de los controles
  // conectados. dataUpdated indica si llegó un paquete NUEVO en este
  // ciclo, y es lo que mantiene vivo el timeout de datos.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    ultimoDatoRecibido = millis();
  }

  revisarBateria();

  // El enlace depende de isConnected() (Bluepad32 detecta la pérdida
  // de conexión de forma confiable). El timeout de datos es OPCIONAL
  // y se usa solo si TIMEOUT_CONTROL_MS > 0. El objeto myController
  // conserva el último estado conocido de sus botones/ejes, así que
  // podemos leerlo en cada ciclo sin esperar un paquete nuevo.
  bool estaConectado = (myController != nullptr && myController->isConnected());
  bool datosRecientes = (TIMEOUT_CONTROL_MS == 0) ||
                        (millis() - ultimoDatoRecibido < TIMEOUT_CONTROL_MS);

  // Operativo solo si: hay enlace AND (timeout desactivado o datos
  // recientes) AND la batería está en buen estado.
  bool operativo = estaConectado && datosRecientes && !bateriaBaja;

  if (operativo) {
    controlConectado = true;
    calcularSalidasDeseadas(myController);
    aplicarInterlockSeguridad();       // única función que escribe DW1-DW4 y DW6
    actualizarSalidasAuxiliares(myController);
    actualizarServo(myController);
  } else {
    // Sin enlace, sin datos recientes o batería baja -> failsafe
    controlConectado = false;
    aplicarFailsafe();
  }

  debugConsola(myController);

  delay(1); // respiro mínimo para el bucle (antes era 10ms)
}

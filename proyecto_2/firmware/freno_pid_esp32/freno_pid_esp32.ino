/*
 * planta_lazo_abierto.ino
 * CE 1110 - Tarea 2, Puntos extra 6 y 7 | ESP32 clasica, Arduino IDE
 * Pieza 1: esqueleto (pines, constantes, PWM motor y servo)
 * Pieza 2: sensor IR por interrupcion, RPM / rad/s y modo p
 * Pieza 3: calibracion de servo (s <deg>) y aplicarFreno(u)
 * Pieza 4: modo manual (m, u) y parada de emergencia (x y boton GPIO33)
 * Pieza 5: ensayos automaticos step / coast-down y registro CSV a 50 Hz
 * Ejemplos: step 150 0 3000 1000 10000     (coast-down)
 *           step 150 0.3 3000 1000 10000   (escalon de frenado)
 * Calibrar angulos y verificar el conteo por vuelta antes de tomar datos reales.
 */

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <errno.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

// ---------- Pines (NO CAMBIAR) ----------
const int PIN_MOTOR_PWM = 25;   // XY-MOS
const int PIN_SERVO     = 26;   // MG996
const int PIN_IR        = 27;   // sensor IR
const int PIN_BTN_START = 32;
const int PIN_BTN_STOP  = 33;

#ifndef USE_PHYSICAL_BUTTONS
#define USE_PHYSICAL_BUTTONS true
#endif

// ---------- Muestreo ----------
const uint32_t TS_US = 20000UL;   // Ts = 20 ms
const uint32_t SERVO_RELEASE_MS = 500; // espera antes de energizar; ajustar al montaje
const uint32_t STOP_HOLD_MS = 500;
const uint32_t MAX_STAGE_MS = 600000;  // limita conversiones y diferencias de tiempo
const uint32_t BUTTON_DEBOUNCE_MS = 40;

// ---------- Sensor de velocidad ----------
const uint32_t PULSES_PER_REV   = 32;        // pulsos efectivos por vuelta configurados
#define        IR_EDGE            FALLING    // contar UN solo tipo de flanco: RISING o FALLING
const uint32_t MIN_PULSE_US     = 2000UL;    // anti-rebote (a 600 RPM, P=32 -> 3125 us)
const uint32_t PULSE_TIMEOUT_US = 250000UL;  // sin pulsos por este tiempo -> 0 RPM
const float    RPM_EMA_ALPHA    = 0.35f;     // filtro EMA (0..1, mayor = menos filtrado)
const uint32_t SPEED_WINDOW_US  = 100000UL;  // ventana del estimador: pulsos de los ultimos 100 ms

// ---------- Motor ----------
const uint32_t MOTOR_PWM_FREQ = 20000;
const uint8_t  MOTOR_PWM_BITS = 8;     // duty 0..255
const int      MOTOR_PWM_MAX  = 255;         // limite de seguridad (bajar para limitar velocidad)
const int      DEFAULT_START_PWM = 150;      // PWM del boton de inicio

// ---------- Servo (valores de relleno, se calibran en la pieza 3) ----------
float SERVO_RELEASE_DEG  = 44.0f;
float SERVO_CONTACT_DEG  = 49.0f;
float SERVO_MAX_SAFE_DEG = 53.0f;
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t  SERVO_BITS    = 14;
const int      SERVO_MIN_US  = 500;
const int      SERVO_MAX_US  = 2500;

// Canales LEDC distintos (timers separados en core 2.x y 3.x)
const int LEDC_CH_SERVO = 0;
const int LEDC_CH_MOTOR = 2;

// ---------- Estado ----------
bool outputsReady = false;
int   motorPwm = 0;
float servoDeg = 20.0f;
float uBrake = 0;   // intensidad de freno 0..1

// Velocidad
float rpmRaw = 0, rpmFilt = 0, radS = 0;

// Modos
enum Mode { MODE_IDLE, MODE_PULSE_CHECK, MODE_SERVO_CAL, MODE_MANUAL, MODE_STEP };
Mode mode = MODE_IDLE;

enum TrialPhase { TRIAL_RELEASE, TRIAL_SPINUP, TRIAL_PRE, TRIAL_BRAKE };
struct Trial {
  TrialPhase phase = TRIAL_RELEASE;
  int pwm = 0;
  float brake = 0;
  uint32_t spinupMs = 0, preMs = 0, durationMs = 0;
  uint32_t phaseStartUs = 0, referenceUs = 0, stoppedSinceUs = 0;
  bool csvOpen = false, speedSeen = false, stopPending = false;
  bool commandRejected = false;
};
Trial trial;
bool manualStartPending = false;
uint32_t manualReleaseUs = 0;

// Manual y botones
int lastManualPwm = DEFAULT_START_PWM;
bool btnStartPrev = false, btnStopPrev = false;
bool btnStartStable = false, startArmed = true;
uint32_t btnStartT = 0;

// Serial (lectura por lineas, sin bloquear)
String rxLine;
bool rxOverflow = false;
bool rxDiscardUntilIdle = false;

// Planificador (Ts con micros(), sin delay)
uint32_t nextTickUs = 0;
uint32_t tickCount = 0;

// ---------- Salida serial no bloqueante ----------
// La UART puede llenarse si la computadora no consume datos. Nunca esperamos
// espacio: una cola acotada absorbe rafagas y abortamos si se pierde telemetria.
const size_t SERIAL_TX_CAPACITY = 4096;
const size_t SERIAL_TX_RESERVE = 256; // garantiza espacio para cerrar CSV y explicar el aborto
char serialTx[SERIAL_TX_CAPACITY];
size_t serialTxHead = 0, serialTxSize = 0;
bool serialOverflow = false;

bool encolarSerial(const char* data, size_t size, bool priority) {
  size_t free = SERIAL_TX_CAPACITY - serialTxSize;
  if (size > free || (!priority && free - size < SERIAL_TX_RESERVE)) {
    serialOverflow = true;
    return false;
  }
  for (size_t i = 0; i < size; i++) {
    serialTx[(serialTxHead + serialTxSize + i) % SERIAL_TX_CAPACITY] = data[i];
  }
  serialTxSize += size;
  return true;
}

void serialPrintf(const char* format, ...) {
  char line[256];
  va_list args;
  va_start(args, format);
  int size = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (size < 0 || (size_t)size >= sizeof(line)) {
    serialOverflow = true;
    return;
  }
  encolarSerial(line, (size_t)size, false);
}

void serialPrintln(const char* line) {
  serialPrintf("%s\n", line);
}

void enviarSerialPendiente() {
  int available = Serial.availableForWrite();
  if (available <= 0 || serialTxSize == 0) return;
  size_t count = serialTxSize;
  if (count > (size_t)available) count = (size_t)available;
  if (count > 64) count = 64; // presupuesto por vuelta del loop
  if (count > SERIAL_TX_CAPACITY - serialTxHead) count = SERIAL_TX_CAPACITY - serialTxHead;
  size_t sent = Serial.write((const uint8_t*)serialTx + serialTxHead, count);
  serialTxHead = (serialTxHead + sent) % SERIAL_TX_CAPACITY;
  serialTxSize -= sent;
}

// ---------- ISR del sensor IR ----------
static portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool isrHasPulse = false;
volatile uint32_t isrLastUs = 0;   // instante del ultimo pulso aceptado
volatile uint32_t isrDtUs = 0;     // tiempo entre los dos ultimos pulsos (0 = invalido)
volatile uint32_t totalPulses = 0;
// Buffer circular con los instantes de los ultimos pulsos aceptados (base del estimador por ventana).
const uint32_t PULSE_RING_SIZE = 32;               // potencia de 2
volatile uint32_t isrRing[PULSE_RING_SIZE];        // micros() de cada pulso aceptado
volatile uint32_t isrRingHead = 0;                 // pulsos escritos desde el ultimo reset

void IRAM_ATTR isrPulse() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&isrMux);
  uint32_t last = isrLastUs;
  uint32_t dt = now - last;
  if (isrHasPulse && dt < MIN_PULSE_US) {          // rebote / ruido
    portEXIT_CRITICAL_ISR(&isrMux);
    return;
  }
  isrDtUs = (isrHasPulse && dt <= PULSE_TIMEOUT_US) ? dt : 0;
  isrHasPulse = true;
  isrLastUs = now;
  totalPulses = totalPulses + 1;
  isrRing[isrRingHead & (PULSE_RING_SIZE - 1)] = now;
  isrRingHead = isrRingHead + 1;
  portEXIT_CRITICAL_ISR(&isrMux);
}

// ---------- PWM compatible con core 2.x y 3.x ----------
bool pwmBegin(int pin, uint32_t freq, uint8_t bits, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  return ledcAttachChannel(pin, freq, bits, ch);
#else
  if (ledcSetup(ch, freq, bits) == 0) return false;
  ledcAttachPin(pin, ch);
  return true;
#endif
}

void pwmWrite(int pin, int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// ---------- Actuadores ----------
void setMotorPWM(int pwm) {
  if (pwm < 0 || !outputsReady) pwm = 0;
  if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
  motorPwm = pwm;
  pwmWrite(PIN_MOTOR_PWM, LEDC_CH_MOTOR, (uint32_t)pwm);
}

void servoWriteDeg(float deg) {
  if (!isfinite(deg) || !outputsReady) {
    setMotorPWM(0);
    return;
  }
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  servoDeg = deg;
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
  uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;
  uint32_t duty = (uint32_t)(us * (float)(1UL << SERVO_BITS) / (float)periodUs);
  pwmWrite(PIN_SERVO, LEDC_CH_SERVO, duty);
}

// u en [0,1]: 0 = liberado; >0 interpola entre CONTACT y MAX_SAFE.
// Funciona con MAX_SAFE > RELEASE o MAX_SAFE < RELEASE.
// Seguridad: con freno aplicado el motor SIEMPRE se apaga.
void aplicarFreno(float u) {
  if (!isfinite(u)) {
    setMotorPWM(0);
    u = 0;
  }
  if (u < 0) u = 0;
  if (u > 1) u = 1;
  uBrake = u;
  if (u <= 0.0f) {
    servoWriteDeg(SERVO_RELEASE_DEG);
  } else {
    setMotorPWM(0);
    servoWriteDeg(SERVO_CONTACT_DEG + u * (SERVO_MAX_SAFE_DEG - SERVO_CONTACT_DEG));
  }
}

bool botonParadaActivo() {
#if USE_PHYSICAL_BUTTONS
  return digitalRead(PIN_BTN_STOP) == HIGH;
#else
  return false;
#endif
}

// Toda salida de un ensayo deja los actuadores en reposo y cierra la captura.
void finalizarOperacion(const char* reason) {
  setMotorPWM(0);
  aplicarFreno(0);
  manualStartPending = false;
  startArmed = false; // un inicio ya sostenido no puede reactivar tras una parada
  btnStartT = millis();
  mode = MODE_IDLE;
  char result[256];
  int size = snprintf(result, sizeof(result), "%s%s#RESULT %s\n",
                      trial.csvOpen ? "#END_CSV\n" : "",
                      trial.commandRejected ? "#WARN comandos ignorados durante ensayo\n" : "", reason);
  if (size > 0 && (size_t)size < sizeof(result)) encolarSerial(result, (size_t)size, true);
  trial.csvOpen = false;
  trial.commandRejected = false;
  serialOverflow = false;
}

void paradaEmergencia() {
  finalizarOperacion("cancelado: motor PWM=0, freno liberado");
}

// ---------- Velocidad ----------
void actualizarVelocidad() {
  uint32_t last, dt, now, head;
  uint32_t ring[PULSE_RING_SIZE];
  bool hasPulse;
  portENTER_CRITICAL(&isrMux);
  now = micros();
  last = isrLastUs;
  dt = isrDtUs;
  hasPulse = isrHasPulse;
  head = isrRingHead;
  if (hasPulse && now - last > PULSE_TIMEOUT_US) {
    // Invalida el historial: evita resucitar una lectura vieja al dar la vuelta micros().
    isrHasPulse = false;
    isrDtUs = 0;
    hasPulse = false;
  }
  for (uint32_t i = 0; i < PULSE_RING_SIZE; i++) ring[i] = isrRing[i];
  portEXIT_CRITICAL(&isrMux);

  // Pulsos dentro de la ventana, del mas nuevo al mas viejo.
  uint32_t avail = head < PULSE_RING_SIZE ? head : PULSE_RING_SIZE;
  uint32_t k = 0, tFirst = 0, tLast = 0;
  if (hasPulse) {
    for (uint32_t i = 0; i < avail; i++) {
      uint32_t t = ring[(head - 1 - i) & (PULSE_RING_SIZE - 1)];
      if (now - t > SPEED_WINDOW_US) break;
      if (k == 0) tLast = t;
      tFirst = t;
      k++;
    }
  }

  uint32_t since = now - last;
  if (!hasPulse) {
    rpmRaw = 0;
  } else if (k >= 2 && tLast != tFirst) {
    // (k-1) intervalos entre el primer y el ultimo pulso de la ventana.
    uint32_t span = tLast - tFirst;
    float meanDt = (float)span / (float)(k - 1);
    // Durante la desaceleracion el ultimo pulso se atrasa: si ya paso mas de 1.5 intervalos
    // se baja la estimacion (continuo, no actua con el jitter normal de los pulsos).
    float cota = 0.67f * (float)since;
    if (cota > meanDt) meanDt = cota;
    rpmRaw = 60.0e6f / ((float)PULSES_PER_REV * meanDt);
  } else if (dt != 0) {
    // Velocidad muy baja: menos de 2 pulsos en la ventana, se usa el ultimo intervalo.
    uint32_t dtEff = dt;
    if (since > dtEff) dtEff = since;
    rpmRaw = 60.0e6f / ((float)PULSES_PER_REV * (float)dtEff);
  } else {
    rpmRaw = 0;
  }
  if (rpmRaw <= 0.0f) rpmFilt = 0;
  else rpmFilt += RPM_EMA_ALPHA * (rpmRaw - rpmFilt);
  radS = rpmFilt * (2.0f * PI / 60.0f);
}

uint32_t leerPulsos() {
  uint32_t n;
  portENTER_CRITICAL(&isrMux);
  n = totalPulses;
  portEXIT_CRITICAL(&isrMux);
  return n;
}

void resetPulsos() {
  portENTER_CRITICAL(&isrMux);
  totalPulses = 0;
  isrRingHead = 0;
  isrHasPulse = false;
  isrLastUs = 0;
  isrDtUs = 0;
  portEXIT_CRITICAL(&isrMux);
  rpmRaw = rpmFilt = radS = 0;
}

// El motor solo arranca tras dar tiempo al servo para liberar la zapata.
void iniciarManual(int pwm) {
  setMotorPWM(0);
  aplicarFreno(0);
  if (pwm > 0) lastManualPwm = pwm;
  manualStartPending = pwm > 0;
  manualReleaseUs = micros();
  mode = MODE_MANUAL;
  serialPrintf("#OK manual: pwm solicitado=%d, esperando liberacion del freno\n", pwm);
}

void iniciarEnsayo(int pwm, float brake, uint32_t spinup, uint32_t pre, uint32_t duration) {
  setMotorPWM(0);
  aplicarFreno(0);
  manualStartPending = false;
  resetPulsos();
  trial = Trial{};
  trial.pwm = pwm;
  trial.brake = brake;
  trial.spinupMs = spinup;
  trial.preMs = pre;
  trial.durationMs = duration;
  trial.phaseStartUs = micros();
  mode = MODE_STEP;
  serialPrintln("#OK ensayo: liberando freno; x cancela en cualquier fase");
}

void registrarMuestra(uint32_t now) {
  // Origen fijo: instante programado del escalon. La primera fila BRAKE
  // indica el corte real, incluido cualquier retraso; no se reinicia el reloj
  // al cambiar de fase porque alteraria los intervalos ya enviados en PRE.
  int32_t relativeUs = (int32_t)(now - trial.referenceUs);
  serialPrintf("%.3f,%s,%d,%.4f,%.2f,%.3f,%.3f,%.4f,%lu\n",
                relativeUs / 1000.0, trial.phase == TRIAL_PRE ? "PRE" : "BRAKE",
                motorPwm, uBrake, servoDeg, rpmRaw, rpmFilt, radS,
                (unsigned long)leerPulsos());
}

void actualizarEnsayo(uint32_t now) {
  if (rpmRaw > 0) trial.speedSeen = true;
  uint32_t elapsed = now - trial.phaseStartUs;
  switch (trial.phase) {
    case TRIAL_RELEASE:
      if (elapsed < SERVO_RELEASE_MS * 1000UL) return;
      setMotorPWM(trial.pwm);
      trial.phase = TRIAL_SPINUP;
      trial.phaseStartUs = now;
      return;

    case TRIAL_SPINUP: {
      if (elapsed < trial.spinupMs * 1000UL) return;
      trial.phase = TRIAL_PRE;
      trial.phaseStartUs = now;
      trial.referenceUs = now + trial.preMs * 1000UL;
      trial.csvOpen = true;
      const char header[] = "#START_CSV\nt_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses\n";
      if (!encolarSerial(header, sizeof(header) - 1, false)) {
        trial.csvOpen = false;
        finalizarOperacion("error_buffer_serial: no se inicio captura");
        return;
      }
      // Continúa para registrar PRE sin perder el primer tick (o frenar si pre=0).
      break;
    }

    case TRIAL_PRE:
    case TRIAL_BRAKE:
      break;
  }

  if (trial.phase == TRIAL_PRE && now - trial.phaseStartUs >= trial.preMs * 1000UL) {
    setMotorPWM(0); // tambien en coast-down: aplicarFreno(0) solo libera
    aplicarFreno(trial.brake);
    trial.phase = TRIAL_BRAKE;
    trial.phaseStartUs = now;
  }
  registrarMuestra(now);
  if (trial.phase != TRIAL_BRAKE) return;

  // Sin una medicion valida previa no se confunde ausencia de sensor con parada.
  if (trial.speedSeen && rpmRaw == 0) {
    if (!trial.stopPending) {
      trial.stopPending = true;
      trial.stoppedSinceUs = now;
    }
    if (now - trial.stoppedSinceUs >= STOP_HOLD_MS * 1000UL) {
      finalizarOperacion("parada_detectada");
      return;
    }
  } else {
    trial.stopPending = false;
  }
  if (now - trial.phaseStartUs >= trial.durationMs * 1000UL) {
    finalizarOperacion(trial.speedSeen ? "duracion_maxima" : "duracion_maxima_sin_velocidad_valida");
  }
}

// ---------- Tick periodico (Ts = 20 ms) ----------
void tickMuestreo() {
  actualizarVelocidad();
  tickCount++;

  uint32_t now = micros();
  if (mode == MODE_STEP) {
    actualizarEnsayo(now);
    return;
  }
  if (manualStartPending && now - manualReleaseUs >= SERVO_RELEASE_MS * 1000UL) {
    manualStartPending = false;
    setMotorPWM(lastManualPwm);
  }

  if (mode == MODE_PULSE_CHECK && tickCount % 10 == 0) {   // cada 200 ms
    uint32_t n = leerPulsos();
    serialPrintf("#P pulsos=%lu vueltas=%.2f rpm=%.1f\n",
                  (unsigned long)n, (float)n / (float)PULSES_PER_REV, rpmFilt);
  }

  if (mode == MODE_MANUAL && tickCount % 5 == 0) {          // telemetria cada 100 ms
    serialPrintf("t_ms=%lu pwm=%d u=%.2f servo=%.1f rpm=%.1f rad_s=%.2f pulsos=%lu\n",
                  (unsigned long)millis(), motorPwm, uBrake, servoDeg,
                  rpmFilt, radS, (unsigned long)leerPulsos());
  }
}

// ---------- Botones (opcionales) ----------
void leerBotones() {
#if USE_PHYSICAL_BUTTONS
  uint32_t now = millis();
  bool inicioPresionado = digitalRead(PIN_BTN_START) == HIGH;
  bool paradaPresionada = botonParadaActivo();

  // La parada actua al primer nivel alto; no espera al antirrebote del inicio.
  if (paradaPresionada && (!btnStopPrev || mode != MODE_IDLE)) paradaEmergencia();
  btnStopPrev = paradaPresionada;
  if (inicioPresionado != btnStartPrev) {
    btnStartPrev = inicioPresionado;
    btnStartT = now;
  }
  if (now - btnStartT >= BUTTON_DEBOUNCE_MS) {
    if (!inicioPresionado) startArmed = true; // tambien se exige una liberacion estable
    if (inicioPresionado != btnStartStable) {
      btnStartStable = inicioPresionado;
      if (inicioPresionado) {
        bool mayStart = startArmed && !paradaPresionada && mode != MODE_STEP && outputsReady &&
                        serialTxSize <= SERIAL_TX_CAPACITY - 2 * SERIAL_TX_RESERVE;
        startArmed = false;
        if (mayStart) iniciarManual(lastManualPwm);
      }
    }
  }
#endif
}

// ---------- Ayuda ----------
void imprimirAyuda() {
  serialPrintln("=== planta_lazo_abierto (ESP32) ===");
  serialPrintf("Pines: motor=GPIO%d servo=GPIO%d IR=GPIO%d inicio=GPIO%d parada=GPIO%d\n",
                PIN_MOTOR_PWM, PIN_SERVO, PIN_IR, PIN_BTN_START, PIN_BTN_STOP);
  serialPrintf("Ts=%lu ms | servo RELEASE=%.1f CONTACT=%.1f MAX_SAFE=%.1f deg\n",
                (unsigned long)(TS_US / 1000UL),
                SERVO_RELEASE_DEG, SERVO_CONTACT_DEG, SERVO_MAX_SAFE_DEG);
  serialPrintf("Sensor: P=%lu pulsos/rev, flanco=%s\n", (unsigned long)PULSES_PER_REV,
                (IR_EDGE == RISING) ? "RISING" : "FALLING");
  serialPrintln("Comandos:");
  serialPrintln("  h            ayuda");
  serialPrintf("  p            verificar IR (1 vuelta = %lu pulsos)\n",
                (unsigned long)PULSES_PER_REV);
  serialPrintln("  s <deg>      calibrar servo 0..180 (motor apagado), de 5 en 5 grados");
  serialPrintln("  m <pwm>      motor a lazo abierto, pwm 0..255, freno liberado");
  serialPrintln("  u <val>      apaga motor y frena con intensidad 0.0..1.0 (0 = liberar)");
  serialPrintln("  step <pwm_ini> <u_freno> <spinup_ms> <pre_ms> <dur_ms>");
  serialPrintln("               CSV a 50 Hz; u=0: coast-down; t=0 programado, primera BRAKE=corte real");
  serialPrintf("               tiempos hasta %lu ms; spinup/dur >=20 ms; pre puede ser 0\n", (unsigned long)MAX_STAGE_MS);
  serialPrintln("  x            parada de emergencia (tambien boton GPIO33)");
}

// ---------- Comandos por Serial ----------
// Divide la linea en tokens separados por espacios
int tokenizar(const String& s, String* out, int maxTok) {
  int n = 0, i = 0, len = s.length();
  while (i < len && n < maxTok) {
    while (i < len && (s[i] == ' ' || s[i] == '\t')) i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s[j] != ' ' && s[j] != '\t') j++;
    out[n++] = s.substring(i, j);
    i = j;
  }
  return n;
}

// Conversion estricta: rechaza texto sobrante, desbordamientos, NaN e infinito.
bool leerReal(const String& text, float minimum, float maximum, float& value) {
  char* end = nullptr;
  errno = 0;
  value = strtof(text.c_str(), &end);
  return end != text.c_str() && end == text.c_str() + text.length() && errno != ERANGE &&
         isfinite(value) && value >= minimum && value <= maximum;
}

bool leerEntero(const String& text, uint32_t minimum, uint32_t maximum, uint32_t& value) {
  if (text.length() == 0) return false;
  for (size_t i = 0; i < text.length(); i++) {
    if (text[i] < '0' || text[i] > '9') return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (errno == ERANGE || *end != '\0' || parsed < minimum || parsed > maximum) return false;
  value = (uint32_t)parsed;
  return true;
}

void procesarComando(String line) {
  line.trim();
  if (line.length() == 0) return;
  String t[7]; // el septimo token permite rechazar argumentos sobrantes
  int n = tokenizar(line, t, 7);
  String cmd = t[0];
  cmd.toLowerCase();

  if (cmd != "x" && mode == MODE_STEP) {
    if (trial.csvOpen) trial.commandRejected = true;
    else serialPrintln("#ERR ensayo activo: use x antes de otro comando");
    return;
  }
  if (cmd != "x" && cmd != "h" && !outputsReady) {
    serialPrintln("#ERR PWM o calibracion invalidos; corrija configuracion y reinicie");
    return;
  }
  if (cmd != "x" && cmd != "h" && serialTxSize > SERIAL_TX_CAPACITY - 2 * SERIAL_TX_RESERVE) {
    serialPrintln("#ERR salida serial congestionada; espere a que se vacie");
    return;
  }
  if (cmd != "x" && cmd != "h" && botonParadaActivo()) {
    serialPrintln("#ERR libere el boton de parada antes de operar");
    return;
  }

  if ((cmd == "h" || cmd == "p" || cmd == "x") && n != 1) {
    serialPrintln("#ERR este comando no acepta argumentos");
    return;
  }

  if (cmd == "x") {
    paradaEmergencia();

  } else if (cmd == "h") {
    imprimirAyuda();

  } else if (cmd == "p") {
    manualStartPending = false;
    setMotorPWM(0);
    aplicarFreno(0);
    resetPulsos();
    mode = MODE_PULSE_CHECK;
    serialPrintf("#OK verificacion IR: gire el disco 1 vuelta a mano; esperado %lu pulsos. 'x' para salir\n",
                  (unsigned long)PULSES_PER_REV);

  } else if (cmd == "s") {
    float deg;
    if (n != 2 || !leerReal(t[1], 0, 180, deg)) { serialPrintln("#ERR uso: s <deg 0..180>"); return; }
    manualStartPending = false;
    setMotorPWM(0);
    mode = MODE_SERVO_CAL;
    uBrake = 0;
    servoWriteDeg(deg);
    serialPrintf("#OK servo en %.1f deg (motor apagado). Anote RELEASE / CONTACT / MAX_SAFE\n", servoDeg);

  } else if (cmd == "m") {
    uint32_t pwm;
    if (n != 2 || !leerEntero(t[1], 0, MOTOR_PWM_MAX, pwm)) { serialPrintln("#ERR uso: m <pwm 0..255>"); return; }
    iniciarManual((int)pwm);

  } else if (cmd == "u") {
    float u;
    if (n != 2 || !leerReal(t[1], 0, 1, u)) { serialPrintln("#ERR uso: u <0.0..1.0>"); return; }
    manualStartPending = false;
    setMotorPWM(0);                  // u=0 tambien corta el motor: coast-down manual
    mode = MODE_MANUAL;
    aplicarFreno(u);                 // si u>0 tambien apaga el motor
    serialPrintf("#OK freno u=%.2f (servo %.1f deg), motor PWM=%d\n", uBrake, servoDeg, motorPwm);

  } else if (cmd == "step") {
    uint32_t pwm, spinup, pre, duration;
    float brake;
    if (n != 6 || !leerEntero(t[1], 1, MOTOR_PWM_MAX, pwm) ||
        !leerReal(t[2], 0, 1, brake) ||
        !leerEntero(t[3], TS_US / 1000, MAX_STAGE_MS, spinup) ||
        !leerEntero(t[4], 0, MAX_STAGE_MS, pre) ||
        !leerEntero(t[5], TS_US / 1000, MAX_STAGE_MS, duration)) {
      serialPrintln("#ERR uso: step <pwm 1..255> <u 0..1> <spinup_ms> <pre_ms> <dur_ms>; h = limites");
      return;
    }
    iniciarEnsayo((int)pwm, brake, spinup, pre, duration);
  } else {
    serialPrintln("#ERR comando desconocido (h = ayuda)");
  }
}

void leerSerial() {
  // Presupuesto por vuelta para no postergar indefinidamente botones y muestreo.
  for (int remaining = 64; remaining > 0 && Serial.available() > 0; remaining--) {
    char c = (char)Serial.read();
    if (rxDiscardUntilIdle) continue;
    if (c == 'x' || c == 'X') { // ningun comando numerico valido contiene x
      rxLine = "";
      rxOverflow = false;
      paradaEmergencia();
      rxDiscardUntilIdle = true; // descarta ordenes que ya estaban en la misma rafaga
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (rxOverflow) {
        if (mode == MODE_STEP && trial.csvOpen) trial.commandRejected = true;
        else serialPrintln("#ERR linea demasiado larga o caracteres invalidos");
      }
      else if (rxLine.length() > 0) procesarComando(rxLine);
      rxLine = "";
      rxOverflow = false;
    } else if ((c < 32 && c != '\t') || (uint8_t)c > 126) {
      rxOverflow = true; // no aceptar NUL, controles ni bytes binarios en comandos
    } else if (!rxOverflow && rxLine.length() < 120) {
      rxLine += c;
    } else {
      rxOverflow = true;
    }
  }
  if (Serial.available() == 0) rxDiscardUntilIdle = false;
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);

  // Mantiene el driver apagado incluso si la asignacion LEDC falla.
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  digitalWrite(PIN_MOTOR_PWM, LOW);
  bool motorReady = pwmBegin(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, LEDC_CH_MOTOR);
  bool servoReady = pwmBegin(PIN_SERVO, SERVO_FREQ_HZ, SERVO_BITS, LEDC_CH_SERVO);
  bool calibrationValid = isfinite(SERVO_RELEASE_DEG) && isfinite(SERVO_CONTACT_DEG) &&
                          isfinite(SERVO_MAX_SAFE_DEG) &&
                          SERVO_RELEASE_DEG >= 0 && SERVO_RELEASE_DEG <= 180 &&
                          SERVO_MAX_SAFE_DEG >= 0 && SERVO_MAX_SAFE_DEG <= 180 &&
                          (SERVO_CONTACT_DEG - SERVO_RELEASE_DEG) *
                          (SERVO_MAX_SAFE_DEG - SERVO_CONTACT_DEG) >= 0;
  outputsReady = motorReady && servoReady && calibrationValid;
  setMotorPWM(0);                    // motor apagado al arrancar
  servoWriteDeg(SERVO_RELEASE_DEG);  // freno liberado al arrancar

#if USE_PHYSICAL_BUTTONS
  pinMode(PIN_BTN_START, INPUT_PULLDOWN);
  pinMode(PIN_BTN_STOP, INPUT_PULLDOWN);
  // estado inicial real: un boton ya presionado al encender NO cuenta como pulsacion
  btnStartPrev = digitalRead(PIN_BTN_START) == HIGH;
  btnStopPrev  = botonParadaActivo();
  btnStartStable = btnStartPrev;
  startArmed = !btnStartPrev;
  btnStartT = millis();
#endif

  pinMode(PIN_IR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IR), isrPulse, IR_EDGE);

  nextTickUs = micros() + TS_US;
  imprimirAyuda();
  if (!outputsReady) serialPrintln("#ERR inicializacion: PWM o calibracion invalidos; actuadores deshabilitados");
}

void loop() {
  leerBotones();

  uint32_t now = micros();
  if ((int32_t)(now - nextTickUs) >= 0) {
    nextTickUs += TS_US;
    // Omite ticks vencidos en vez de ejecutar muestras artificiales en rafaga.
    if ((int32_t)(now - nextTickUs) >= 0) nextTickUs = now + TS_US;
    tickMuestreo();
  }
  leerSerial();
  if (serialOverflow) {
    if (mode != MODE_IDLE) finalizarOperacion("error_buffer_serial: captura incompleta");
    else serialOverflow = false; // no llenar la reserva con errores repetidos estando en reposo
  }
  enviarSerialPendiente();
}

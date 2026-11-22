/*
 * planta_lazo_abierto.ino
 * CE 1110 - Tarea 2, Puntos extra 6 y 7 | ESP32 clasica, Arduino IDE
 * Pieza 1: esqueleto (pines, constantes, PWM motor y servo)
 * Pieza 2: sensor IR por interrupcion, RPM / rad/s y modo p
 * Pieza 3: calibracion de servo (s <deg>) y aplicarFreno(u)
 * Pieza 4: modo manual (m, u) y parada de emergencia (x y boton GPIO33)
 */

#include <Arduino.h>

// ---------- Pines (NO CAMBIAR) ----------
const int PIN_MOTOR_PWM = 25;   // XY-MOS
const int PIN_SERVO     = 26;   // MG996
const int PIN_IR        = 27;   // sensor IR
const int PIN_BTN_START = 32;
const int PIN_BTN_STOP  = 33;

#define USE_PHYSICAL_BUTTONS true

// ---------- Muestreo ----------
const uint32_t TS_US = 20000UL;   // Ts = 20 ms

// ---------- Sensor de velocidad ----------
const uint32_t PULSES_PER_REV   = 16;        // 16 bloques negros + 16 espacios
#define        IR_EDGE            FALLING    // contar UN solo tipo de flanco: RISING o FALLING
const uint32_t MIN_PULSE_US     = 2000UL;    // anti-rebote (a 600 RPM, P=16 -> 6250 us)
const uint32_t PULSE_TIMEOUT_US = 250000UL;  // sin pulsos por este tiempo -> 0 RPM
const float    RPM_EMA_ALPHA    = 0.35f;     // filtro EMA (0..1, mayor = menos filtrado)

// ---------- Motor ----------
const uint32_t MOTOR_PWM_FREQ = 20000;
const uint8_t  MOTOR_PWM_BITS = 8;     // duty 0..255
const int      MOTOR_PWM_MAX  = 255;         // limite de seguridad (bajar para limitar velocidad)
const int      DEFAULT_START_PWM = 150;      // PWM del boton de inicio

// ---------- Servo (valores de relleno, se calibran en la pieza 3) ----------
float SERVO_RELEASE_DEG  = 20.0f;
float SERVO_CONTACT_DEG  = 50.0f;
float SERVO_MAX_SAFE_DEG = 75.0f;
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t  SERVO_BITS    = 14;
const int      SERVO_MIN_US  = 500;
const int      SERVO_MAX_US  = 2500;

// Canales LEDC distintos (timers separados en core 2.x y 3.x)
const int LEDC_CH_SERVO = 0;
const int LEDC_CH_MOTOR = 2;

// ---------- Estado ----------
int   motorPwm = 0;
float servoDeg = 20.0f;
float uBrake = 0;   // intensidad de freno 0..1

// Velocidad
float rpmRaw = 0, rpmFilt = 0, radS = 0;

// Modos
enum Mode { MODE_IDLE, MODE_PULSE_CHECK, MODE_SERVO_CAL, MODE_MANUAL };
Mode mode = MODE_IDLE;

// Manual y botones
int lastManualPwm = DEFAULT_START_PWM;
bool btnStartPrev = true, btnStopPrev = true;
uint32_t btnStartT = 0, btnStopT = 0;

// Serial (lectura por lineas, sin bloquear)
String rxLine;

// Planificador (Ts con micros(), sin delay)
uint32_t nextTickUs = 0;
uint32_t tickCount = 0;

// ---------- ISR del sensor IR ----------
static portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrLastUs = 0;   // instante del ultimo pulso aceptado
volatile uint32_t isrDtUs = 0;     // tiempo entre los dos ultimos pulsos (0 = invalido)
volatile uint32_t totalPulses = 0;

void IRAM_ATTR isrPulse() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&isrMux);
  uint32_t last = isrLastUs;
  uint32_t dt = now - last;
  if (last != 0 && dt < MIN_PULSE_US) {          // rebote / ruido
    portEXIT_CRITICAL_ISR(&isrMux);
    return;
  }
  isrDtUs = (last != 0 && dt <= PULSE_TIMEOUT_US) ? dt : 0;
  isrLastUs = now;
  totalPulses++;
  portEXIT_CRITICAL_ISR(&isrMux);
}

// ---------- PWM compatible con core 2.x y 3.x ----------
void pwmBegin(int pin, uint32_t freq, uint8_t bits, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttachChannel(pin, freq, bits, ch);
#else
  ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
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
  if (pwm < 0) pwm = 0;
  if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
  motorPwm = pwm;
  pwmWrite(PIN_MOTOR_PWM, LEDC_CH_MOTOR, (uint32_t)pwm);
}

void servoWriteDeg(float deg) {
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

void paradaEmergencia() {
  setMotorPWM(0);
  aplicarFreno(0);
  mode = MODE_IDLE;
  Serial.println("#OK parada: motor PWM=0, freno liberado");
}

// ---------- Velocidad ----------
void actualizarVelocidad() {
  uint32_t last, dt;
  portENTER_CRITICAL(&isrMux);
  last = isrLastUs;
  dt = isrDtUs;
  portEXIT_CRITICAL(&isrMux);

  uint32_t now = micros();
  uint32_t since = now - last;

  if (last == 0 || since > PULSE_TIMEOUT_US || dt == 0) {
    rpmRaw = 0;
  } else {
    uint32_t dtEff = dt;
    if (since > dtEff) dtEff = since;   // en frenado: cota superior, evita valor congelado
    rpmRaw = 60.0e6f / ((float)PULSES_PER_REV * (float)dtEff);
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
  isrLastUs = 0;
  isrDtUs = 0;
  portEXIT_CRITICAL(&isrMux);
}

// ---------- Tick periodico (Ts = 20 ms) ----------
void tickMuestreo() {
  actualizarVelocidad();
  tickCount++;

  if (mode == MODE_PULSE_CHECK && tickCount % 10 == 0) {   // cada 200 ms
    uint32_t n = leerPulsos();
    Serial.printf("#P pulsos=%lu vueltas=%.2f rpm=%.1f\n",
                  (unsigned long)n, (float)n / (float)PULSES_PER_REV, rpmFilt);
  }

  if (mode == MODE_MANUAL && tickCount % 5 == 0) {          // telemetria cada 100 ms
    Serial.printf("t_ms=%lu pwm=%d u=%.2f servo=%.1f rpm=%.1f rad_s=%.2f pulsos=%lu\n",
                  (unsigned long)millis(), motorPwm, uBrake, servoDeg,
                  rpmFilt, radS, (unsigned long)leerPulsos());
  }
}

// ---------- Botones (opcionales) ----------
void leerBotones() {
#if USE_PHYSICAL_BUTTONS
  uint32_t now = millis();
  bool bStart = digitalRead(PIN_BTN_START);
  bool bStop  = digitalRead(PIN_BTN_STOP);

  if (bStop != btnStopPrev && (now - btnStopT) > 40) {
    btnStopT = now;
    btnStopPrev = bStop;
    if (!bStop) paradaEmergencia();                     // flanco de bajada = pulsado
  }
  if (bStart != btnStartPrev && (now - btnStartT) > 40) {
    btnStartT = now;
    btnStartPrev = bStart;
    if (!bStart && bStop) {                             // inicio no actua si parada esta presionado
      aplicarFreno(0);
      setMotorPWM(lastManualPwm);
      mode = MODE_MANUAL;
      Serial.printf("#OK boton inicio: manual pwm=%d\n", lastManualPwm);
    }
  }
#endif
}

// ---------- Ayuda ----------
void imprimirAyuda() {
  Serial.println("=== planta_lazo_abierto (ESP32) ===");
  Serial.printf("Pines: motor=GPIO%d servo=GPIO%d IR=GPIO%d inicio=GPIO%d parada=GPIO%d\n",
                PIN_MOTOR_PWM, PIN_SERVO, PIN_IR, PIN_BTN_START, PIN_BTN_STOP);
  Serial.printf("Ts=%lu ms | servo RELEASE=%.1f CONTACT=%.1f MAX_SAFE=%.1f deg\n",
                (unsigned long)(TS_US / 1000UL),
                SERVO_RELEASE_DEG, SERVO_CONTACT_DEG, SERVO_MAX_SAFE_DEG);
  Serial.printf("Sensor: P=%lu pulsos/rev, flanco=%s\n", (unsigned long)PULSES_PER_REV,
                (IR_EDGE == RISING) ? "RISING" : "FALLING");
  Serial.println("Comandos:");
  Serial.println("  h            ayuda");
  Serial.println("  p            verificar IR (1 vuelta = 16 pulsos)");
  Serial.println("  s <deg>      calibrar servo 0..180 (motor apagado), de 5 en 5 grados");
  Serial.println("  m <pwm>      motor a lazo abierto, pwm 0..255, freno liberado");
  Serial.println("  u <val>      apaga motor y frena con intensidad 0.0..1.0 (0 = liberar)");
  Serial.println("  x            parada de emergencia (tambien boton GPIO33)");
}

// ---------- Comandos por Serial ----------
// Divide la linea en tokens separados por espacios
int tokenizar(const String& s, String* out, int maxTok) {
  int n = 0, i = 0, len = s.length();
  while (i < len && n < maxTok) {
    while (i < len && s[i] == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s[j] != ' ') j++;
    out[n++] = s.substring(i, j);
    i = j;
  }
  return n;
}

// true si el texto es un numero (digitos, un punto, signo opcional)
bool esNumero(const String& s) {
  bool digito = false, punto = false;
  for (int i = 0; i < (int)s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') digito = true;
    else if (c == '.' && !punto) punto = true;
    else if (c == '-' && i == 0) continue;
    else return false;
  }
  return digito;
}

// true si el texto es un entero sin signo
bool esEntero(const String& s) {
  return esNumero(s) && s.indexOf('.') < 0 && s.indexOf('-') < 0;
}

void procesarComando(String line) {
  line.trim();
  if (line.length() == 0) return;
  String t[6];
  int n = tokenizar(line, t, 6);
  String cmd = t[0];
  cmd.toLowerCase();

  if (cmd == "x") {
    paradaEmergencia();

  } else if (cmd == "h") {
    imprimirAyuda();

  } else if (cmd == "p") {
    setMotorPWM(0);
    aplicarFreno(0);
    resetPulsos();
    mode = MODE_PULSE_CHECK;
    Serial.printf("#OK verificacion IR: gire el disco 1 vuelta a mano; esperado %lu pulsos. 'x' para salir\n",
                  (unsigned long)PULSES_PER_REV);

  } else if (cmd == "s") {
    if (n < 2 || !esNumero(t[1])) { Serial.println("#ERR uso: s <deg>"); return; }
    float deg = t[1].toFloat();
    if (deg < 0 || deg > 180) { Serial.println("#ERR deg fuera de 0..180"); return; }
    setMotorPWM(0);
    mode = MODE_SERVO_CAL;
    uBrake = 0;
    servoWriteDeg(deg);
    Serial.printf("#OK servo en %.1f deg (motor apagado). Anote RELEASE / CONTACT / MAX_SAFE\n", servoDeg);

  } else if (cmd == "m") {
    if (n < 2 || !esEntero(t[1])) { Serial.println("#ERR uso: m <pwm 0..255>"); return; }
    int pwm = t[1].toInt();
    if (pwm > MOTOR_PWM_MAX) { Serial.printf("#ERR pwm fuera de 0..%d\n", MOTOR_PWM_MAX); return; }
    aplicarFreno(0);                 // freno liberado antes de energizar el motor
    setMotorPWM(pwm);
    if (pwm > 0) lastManualPwm = pwm;
    mode = MODE_MANUAL;
    Serial.printf("#OK manual: pwm=%d, freno liberado\n", pwm);

  } else if (cmd == "u") {
    if (n < 2 || !esNumero(t[1])) { Serial.println("#ERR uso: u <0.0..1.0>"); return; }
    float u = t[1].toFloat();
    if (u < 0 || u > 1) { Serial.println("#ERR u fuera de 0.0..1.0"); return; }
    mode = MODE_MANUAL;
    aplicarFreno(u);                 // si u>0 tambien apaga el motor
    Serial.printf("#OK freno u=%.2f (servo %.1f deg), motor PWM=%d\n", uBrake, servoDeg, motorPwm);

  } else {
    Serial.println("#ERR comando desconocido (h = ayuda)");
  }
}

void leerSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if ((c == 'x' || c == 'X') && rxLine.length() == 0) {   // emergencia inmediata
      paradaEmergencia();
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (rxLine.length() > 0) { procesarComando(rxLine); rxLine = ""; }
    } else if (rxLine.length() < 80) {
      rxLine += c;
    }
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);

  pwmBegin(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, LEDC_CH_MOTOR);
  pwmBegin(PIN_SERVO, SERVO_FREQ_HZ, SERVO_BITS, LEDC_CH_SERVO);
  setMotorPWM(0);                    // motor apagado al arrancar
  servoWriteDeg(SERVO_RELEASE_DEG);  // freno liberado al arrancar

#if USE_PHYSICAL_BUTTONS
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  // estado inicial real: un boton ya presionado al encender NO cuenta como pulsacion
  btnStartPrev = digitalRead(PIN_BTN_START);
  btnStopPrev  = digitalRead(PIN_BTN_STOP);
#endif

  pinMode(PIN_IR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IR), isrPulse, IR_EDGE);

  nextTickUs = micros() + TS_US;
  imprimirAyuda();
}

void loop() {
  leerSerial();
  leerBotones();

  uint32_t now = micros();
  if ((int32_t)(now - nextTickUs) >= 0) {
    nextTickUs += TS_US;
    if ((int32_t)(now - nextTickUs) >= (int32_t)(5 * TS_US)) nextTickUs = now + TS_US;  // resincroniza
    tickMuestreo();
  }
}

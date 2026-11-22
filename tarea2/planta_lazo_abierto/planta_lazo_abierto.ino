/*
 * planta_lazo_abierto.ino
 * CE 1110 - Tarea 2, Puntos extra 6 y 7 | ESP32 clasica, Arduino IDE
 * Pieza 1: esqueleto (pines, constantes, PWM motor y servo)
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

// ---------- Motor ----------
const uint32_t MOTOR_PWM_FREQ = 20000;
const uint8_t  MOTOR_PWM_BITS = 8;     // duty 0..255
const int      MOTOR_PWM_MAX  = 255;

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

// ---------- Ayuda ----------
void imprimirAyuda() {
  Serial.println("=== planta_lazo_abierto (ESP32) ===");
  Serial.printf("Pines: motor=GPIO%d servo=GPIO%d IR=GPIO%d inicio=GPIO%d parada=GPIO%d\n",
                PIN_MOTOR_PWM, PIN_SERVO, PIN_IR, PIN_BTN_START, PIN_BTN_STOP);
  Serial.printf("Ts=%lu ms | servo RELEASE=%.1f CONTACT=%.1f MAX_SAFE=%.1f deg\n",
                (unsigned long)(TS_US / 1000UL),
                SERVO_RELEASE_DEG, SERVO_CONTACT_DEG, SERVO_MAX_SAFE_DEG);
  Serial.println("Comandos: h");
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);

  pwmBegin(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, LEDC_CH_MOTOR);
  pwmBegin(PIN_SERVO, SERVO_FREQ_HZ, SERVO_BITS, LEDC_CH_SERVO);
  setMotorPWM(0);                    // motor apagado al arrancar
  servoWriteDeg(SERVO_RELEASE_DEG);  // freno liberado al arrancar

  imprimirAyuda();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'h' || c == 'H') imprimirAyuda();
  }
}

#include "FastLED.h"

//--------LED----------
#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
int r = 0, g = 0, b = 0;
//--------------------

//-------Ultrasonido--------------
#define TRIG_PIN 13
#define ECHO_PIN 12
//--------------------------------

//-------Inflarrojo---------------
#define PIN_ITR20001_LEFT A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT A0
// CON 0.3 TAMBIEN VA Y 0.4 KD
#define KP 0.245
#define KD 0.32

// Umbral de detección de línea
// Cuando detecta por ejemplo blanco el valor está entre 800 y 900
// pero cuando detecta negro el valor baja mucho. Ponemos este umbral:
const int threshold = 500;  // 700

int left_val;
int right_val;
int mid_val;
int error;
int derivativo;
int error_anterior;
int velocidad;
int anterior;
//--------------------------------

//-------Motor------------------
// Enable/Disable motor control.
//  HIGH: motor control enabled
//  LOW: motor control disabled
#define PIN_Motor_STBY 3

// Group A Motors (Right Side)
// PIN_Motor_AIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_AIN_1 7
// PIN_Motor_PWMA: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMA 5

// Group B Motors (Left Side)
// PIN_Motor_BIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_BIN_1 8
// PIN_Motor_PWMB: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMB 6

// aqui tenia 90
const int i_show_speed_linear = 90;

// aqui tenia 120
const int i_show_speed_angular = 120;
//-----------------------------

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

// Function to control motors
void motorControl(bool motorAForward, int speedA, bool motorBForward, int speedB) {
  // Control motor A (RIGHT)
  digitalWrite(PIN_Motor_AIN_1, motorAForward ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, speedA);

  // Control motor B (LEFT)
  digitalWrite(PIN_Motor_BIN_1, motorBForward ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMB, speedB);
}

void sensorReading() {
  left_val = analogRead(A2);  // read from left Sensor
  right_val = analogRead(A0);
  mid_val = analogRead(A1);
}

void turnLeft(int vel_left) {
  // LED RED
  r = 0;
  g = 255;
  b = 0;
  FastLED.showColor(Color(r, g, b));
  // move
  motorControl(true, vel_left, true, vel_left / 3);
}
void turnRight(int vel_rigth) {
  // LED RED
  r = 0;
  g = 255;
  b = 0;
  FastLED.showColor(Color(r, g, b));
  // move
  motorControl(true, vel_rigth / 3, true, vel_rigth);
}
void forward(int speed) {
  // LED GREEN
  r = 0;
  g = 255;
  b = 0;
  FastLED.showColor(Color(r, g, b));
  // move
  motorControl(true, speed, true, speed);
}
void recovery() {
  r = 255;
  g = 0;
  b = 0;

  FastLED.showColor(Color(r, g, b));
  if (anterior == 1) {
    motorControl(false, 0, true, i_show_speed_angular * 1.5);
  } else if (anterior == 2) {
    motorControl(true, i_show_speed_angular * 1.5, false, 0);
  }
}
void stop_motors() {
  // LED Blue
  r = 0;
  g = 0;
  b = 255;
  FastLED.showColor(Color(r, g, b));
  motorControl(false, 0, false, 0);
  delay(3000);
}

// Define a global variable to store the start time of recovery
// unsigned long recoveryStartTime = 0;
// bool isRecovering = false;
// void recovery() {
//  if (!isRecovering) {
// Set the start time only when entering the recovery mode
// recoveryStartTime = millis();
// isRecovering = true;
//}

// Calculate the elapsed time since the recovery started
// unsigned long elapsedTime = millis() - recoveryStartTime;

// delay(elapsedTime)
// motorControl(false, i_show_speed_linear, false, i_show_speed_linear);
//}

int ping(int TriggerPin, int EchoPin) {
  long duration, distanceCm;

  digitalWrite(TriggerPin, LOW);  // para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  // generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);

  duration = pulseIn(EchoPin, HIGH);  // medimos el tiempo entre pulsos, en microsegundos

  distanceCm = duration * 10 / 292 / 2;  // convertimos a distancia, en cm
  return distanceCm;
}

void setup() {
  // ultrasonidos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // inflarrojos
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);
  // motors
  //  Turn on the engines !!!!
  digitalWrite(PIN_Motor_STBY, HIGH);
  // Set motor control pins as outputs
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  // LED
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  Serial.begin(9600);  // // Serial Communication is starting with 9600 of baudrate speed
}

void loop() {
  sensorReading();
  //int distance = ping(TRIG_PIN, ECHO_PIN);

  error = abs(left_val - right_val);
  derivativo = error - error_anterior;
  velocidad = KP * error + KD * derivativo;
  error_anterior = error;

  if (right_val >= threshold) {  // RIGHT
    anterior = 1;
    turnRight(velocidad);

  } else if (left_val >= threshold) {  // LEFT
    anterior = 2;
    turnLeft(velocidad);

  } else if (mid_val >= threshold) {  // FORWARD
    // tenia 100 y va bien
    forward(130);
  } else {
    r = 255;
    g = 0;
    b = 0;

    FastLED.showColor(Color(r, g, b));
  }
  // else {
  //     recovery();
  // }

  //if (distance < 10) {  // STOP
    //stop_motors();
  //}
  // if (left_val <= threshold && right_val <= threshold && mid_val <= threshold) {
  //   recovery();
  // }
}

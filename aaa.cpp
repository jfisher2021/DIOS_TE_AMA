#include "FastLED.h"

//--------LED----------
#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
int r=0,g=0,b=0;
//--------------------


//-------Ultrasonido--------------
#define TRIG_PIN 13  
#define ECHO_PIN 12  
//--------------------------------

//-------Inflarrojo---------------
#define PIN_ITR20001_LEFT   A2
#define PIN_ITR20001_MIDDLE A1
#define PIN_ITR20001_RIGHT  A0

// Umbral de detección de línea
//Cuando detecta por ejemplo blanco el valor está entre 800 y 900 
//pero cuando detecta negro el valor baja mucho. Ponemos este umbral:
const int threshold = 633;//700

int left_val;
int right_val;
int mid_val;
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

const int i_show_speed_linear = 100;
const int i_show_speed_angular = 150;
//-----------------------------


uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
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


void sensorReading(){
  left_val = analogRead(A2); // read from left Sensor
  right_val = analogRead(A0);
  mid_val =  analogRead(A1);
}

void turnLeft(){
  //LED RED
  r=255;
  g=0;
  b=0;
  FastLED.showColor(Color(r, g, b));
  //move
  motorControl(true,i_show_speed_angular,false,0);
}
void turnRight(){
  //LED RED
  r=255;
  g=0;
  b=0;
  FastLED.showColor(Color(r, g, b));
  //move
  motorControl(false,0,true,i_show_speed_angular);
}
void forward(){
  //LED GREEN
  r=0;
  g=255;
  b=0;
  FastLED.showColor(Color(r, g, b));
  //move
  motorControl(true,i_show_speed_linear,true,i_show_speed_linear);
}
void stop_motors(){
  //LED Blue
  r=0;
  g=0;
  b=255;
  FastLED.showColor(Color(r, g, b));
  motorControl(false,0,false,0);
  delay(10000);
}


// Define a global variable to store the start time of recovery
//unsigned long recoveryStartTime = 0;
//bool isRecovering = false;
//void recovery() {
//  if (!isRecovering) {
    // Set the start time only when entering the recovery mode
    //recoveryStartTime = millis();
    //isRecovering = true;
  //}

  // Calculate the elapsed time since the recovery started
  //unsigned long elapsedTime = millis() - recoveryStartTime;

  //delay(elapsedTime)
  //motorControl(false, i_show_speed_linear, false, i_show_speed_linear);
//}

int ping(int TriggerPin, int EchoPin) {
  long duration, distanceCm;
  
  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  
  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
  return distanceCm;
}


// Variables del controlador PID
double kp = 0.5;  // Ganancia proporcional
double ki = 0.2;  // Ganancia integral
double kd = 0.1;  // Ganancia derivativa

double setpoint = 0;  // Valor objetivo
double input = 0;     // Valor actual
double output = 0;    // Valor de salida

double error = 0;           // Error actual
double last_error = 0;      // Error anterior
double integral = 0;        // Término integral
double derivative = 0;      // Término derivativo

unsigned long last_time = 0;  // Último tiempo de actualización

void setup() {
  //ultrasonidos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  //inflarrojos
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);
  //motors
  // Turn on the engines !!!!
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


  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop() {
  // Actualizar el valor actual (input)
  sensorReading();
  int distance = ping(TRIG_PIN, ECHO_PIN);
  input = distance;

  // Calcular el error
  error = setpoint - input;

  // Calcular el término integral
  integral += error;

  // Calcular el término derivativo
  unsigned long current_time = millis();
  double dt = (current_time - last_time) / 1000.0;  // Tiempo transcurrido en segundos
  derivative = (error - last_error) / dt;

  // Calcular la salida del controlador PID
  output = kp * error + ki * integral + kd * derivative;

  // Aplicar la salida a los motores
  if (output > 0) {
    motorControl(true, output, true, output);
  } else {
    motorControl(false, -output, false, -output);
  }

  if (distance < 10) {
    stop_motors();
  }
  
  // Actualizar el tiempo y el error anterior
  last_time = current_time;
  last_error = error;

}

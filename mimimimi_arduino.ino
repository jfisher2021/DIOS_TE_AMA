#include "FastLED.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Arduino_FreeRTOS.h>

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
// Negro 1024 y blanco 0 va entre esos valores
const int threshold = 500;  // 700

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

#define SPEED_LINEAR 130
// aqui tenia 120
const int i_show_speed_angular = 120;
//-----------------------------

int left_val, right_val, mid_val, error, derivativo, error_anterior, velocidad, anterior;
long distance;
Thread temporary_check_thread = Thread();
ThreadController controller = ThreadController();
Thread distance_thread = Thread();


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

void stop_motors() {
    // LED Blue
    r = 0;
    g = 0;
    b = 255;
    FastLED.showColor(Color(r, g, b));
    motorControl(false, 0, false, 0);
    delay(3000);
}

void distance_ping() {
    long duration;

    digitalWrite(TRIG_PIN, LOW);  // para generar un pulso limpio ponemos a LOW 4us
    delayMicroseconds(4);
    digitalWrite(TRIG_PIN, HIGH);  // generamos Trigger (disparo) de 10us
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);  // medimos el tiempo entre pulsos, en microsegundos

    distance = duration * 10 / 292 / 2;  // convertimos a distancia, en cm
}

void ping_time_check(){
  long time = millis();
  Serial.print("{PING}" + String(time) + "*");
}

void recovery() {

  if (anterior == 1) {
    motorControl(false, 0, true, i_show_speed_angular * 1.5);
  } else if (anterior == 2) {
    motorControl(true, i_show_speed_angular * 1.5, false, 0);
  }
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

    
    distance_thread.enabled = true;
    distance_thread.setInterval(100);
    distance_thread.onRun(distance_ping);
    controller.add(&distance_thread);
    temporary_check_thread.enabled = true;
    temporary_check_thread.setInterval(4000);
    temporary_check_thread.onRun(ping_time_check);
    controller.add(&temporary_check_thread);

    
    // LED
    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(20);

    Serial.begin(9600);  // // Serial Communication is starting with 9600 of baudrate speed

    String sendBuff;

    Serial.print("{CONNECT}");

    while(1) {
      if (Serial.available()) {
        char c = Serial.read();
        sendBuff += c;
        if (c == '}')  {            
          if(sendBuff == "{CONNECTED}"){
            break;
          }
          sendBuff = "";
        } 
      }
    }
    Serial.print("{START_LAP}");
    unsigned long time_start = millis();


    //xTaskCreate(pid_track, "pid_track", 100, NULL, 2, NULL);
    //xTaskCreate(distance_ping, "CheckDistance", 100, NULL, 1, NULL);
}

void loop() {
    controller.run();
    sensorReading();

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
        forward(SPEED_LINEAR);
    } else {
        r = 255;
        g = 0;
        b = 0;

        FastLED.showColor(Color(r, g, b));
        recovery();

    }

    if (distance < 10) {  // STOP
      stop_motors();
    }

}
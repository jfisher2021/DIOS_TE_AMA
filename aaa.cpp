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

//-------Constantes---------------
#define DISTANCIA 10
#define LINEA 700
#define MITAD_LINEA 250

// Umbral de detección de línea
// Cuando detecta por ejemplo blanco el valor está entre 800 y 900
// pero cuando detecta negro el valor baja mucho. Ponemos este umbral:

// con 700 - 800 esta en linea
// con 200-500 esta entrando el linea (0 saliendo)

int left_val;
int right_val;
int mid_val;
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

const int i_show_speed_linear = 80;
const int i_show_speed_angular = 150;
const int giro_poco = 70;

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

void turnLeft(int valor_left) {
    // LED RED
    r = 255;
    g = 0;
    b = 0;
    FastLED.showColor(Color(r, g, b));
    // move
    motorControl(true, valor_left, false, 0);
}
void turnRight(int valor_right) {
    // LED RED
    r = 255;
    g = 0;
    b = 0;
    FastLED.showColor(Color(r, g, b));
    // move
    motorControl(false, 0, true, valor_right);
}
void forward(int valor_forward) {
    // LED GREEN
    r = 0;
    g = 255;
    b = 0;
    FastLED.showColor(Color(r, g, b));
    // move
    motorControl(true, valor_forward, true, valor_forward);
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


void recovery(int anterior_val) {
    switch (anterior_val) {
        case 1:
            turnRight(i_show_speed_angular);
            break;
        case 2:
            turnLeft(i_show_speed_angular);
            break;
        default:
            break;
    }
}

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
    int distance = ping(TRIG_PIN, ECHO_PIN);
    // Serial.println(distance);
    Serial.print("left_val       ");

    Serial.println(left_val);
    Serial.print("rigth_val    ");

    Serial.println(right_val);
    Serial.print("mid_val         ");

    Serial.println(mid_val);

    if (right_val >= MITAD_LINEA) {  // RIGHT
        anterior = 1;
        if (right_val >= LINEA) {
            turnRight(i_show_speed_angular);
        } else {
            turnRight(giro_poco);
        }
    } else if (left_val >= MITAD_LINEA) {  // LEFTç
        anterior = 2;
        if (left_val >= LINEA) {
            turnLeft(i_show_speed_angular);
        } else {
            turnLeft(giro_poco);
        }
    } else if (mid_val >= LINEA) {  // FORWARD
        forward(i_show_speed_linear);
    } else if (left_val >= LINEA && right_val >= LINEA && mid_val >= LINEA) {
        recovery(anterior);
    }

    if (distance < DISTANCIA) {  // STOP
        stop_motors();
    }
}

#include "FastLED.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


//-----------MQTT-----------------
// Configuración de WiFi
const char *ssid = "TU_SSID_WIFI";
const char *password = "TU_CONTRASEÑA_WIFI";

// Configuración del servidor MQTT
const char *mqtt_server = "193.147.53.2";
const int mqtt_port = 21883;
const char *mqtt_topic = "/SETR/2023/$ID_EQUIPO/";

// ID de equipo y nombre de equipo
const char *team_id = "TU_ID_EQUIPO";
const char *team_name = "TU_NOMBRE_DE_EQUIPO";

// Cliente MQTT
WiFiClient espClient;
PubSubClient client(espClient);

//---------------------------






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
const int threshold = 700;

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
  motorControl(true,175,false,0);
}
void turnRight(){
  //LED RED
  r=255;
  g=0;
  b=0;
  FastLED.showColor(Color(r, g, b));
  //move
  motorControl(false,0,true,175);
}
void forward(){
  //LED GREEN
  r=0;
  g=255;
  b=0;
  FastLED.showColor(Color(r, g, b));
  //move
  motorControl(true,175,true,175);
}
void stop_motors(){
  motorControl(false,0,false,0);
}
//void recovery(){
//  motorControl(false,0,false,200);
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

void reconnect() {
  // Bucle hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");

    // Intentar conectar
    if (client.connect("ArduinoClient")) {
      Serial.println("Conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void init_wifi() {

  // Conectar a la red WiFi
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Conectado a la red WiFi");
}

void setup() {

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

  //Start up wifi
  init_wifi();
  client.setServer(mqtt_server, mqtt_port);
  
  //ultrasonidos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  //inflarrojos
  pinMode(PIN_ITR20001_LEFT, INPUT);
  pinMode(PIN_ITR20001_MIDDLE, INPUT);
  pinMode(PIN_ITR20001_RIGHT, INPUT);
  //motors
  // Gentlemen, start on the engines !!!!
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
}



void loop() {
  //Check connection
  if (!client.connected()) {
    reconnect();
  }

  // Enviar el mensaje de inicio de vuelta una vez (puede ser en setup() si solo debe enviar una vez)
  static bool lapStarted = false;
  if (!lapStarted) {
    String payload = String("{\"team_name\":\"") + team_name + "\",\"id\":\"" + team_id + "\",\"action\":\"START_LAP\"}";
    client.publish(mqtt_topic, payload.c_str());
    lapStarted = true;
  }

  //mantener activa la conexión al servidor MQT
  client.loop();
  
  int distance = ping(TRIG_PIN,ECHO_PIN);

  sensorReading();
  if(left_val <= threshold && right_val >= threshold){//LEFT
    turnLeft();
  }else if(left_val >= threshold && right_val <= threshold ){//RIGHT
    turnRight();
  }else if(left_val <= threshold && right_val <= threshold){//FORWARD
    forward();
  }else if(distance < 10){//STOP
    stop_motors();
  }//else if(leftValue == 1 && rightValue == 1){
    //recovery();
  //}
  delay(50);
}

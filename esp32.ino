#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char* ssid = "LosPollosHermanos";
const char* password = "tdko9519";

#define MQTT_SERVER     "garceta.tsc.urjc.es"
#define MQTT_PORT       21883
#define MQTT_USERNAME   "Dios_te_ama"
#define MQTT_KEY        "33"

const char* topic = "/SETR/2023/3/";
const char* id_equipo = "3";

unsigned long TimeStart = 0;
unsigned long lastPingTime = 0; 


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);

Adafruit_MQTT_Publish   myFeed = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "/feeds/myfeed");


void connectToWiFi() {
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  //Serial.print("IP Address: ");
  //Serial.println(WiFi.localIP());
  //Serial.print("RRSI: ");
  //Serial.println(WiFi.RSSI());
}

void connectToMQTT() {
  // Connect to MQTT Broker
  while (!mqtt.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqtt.connect()) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.println("Failed to connect to MQTT, retrying in 5 seconds...");
      delay(5000);
    }
  }
}
// Publish data to MQTT feed
void publishData(const char* data) {
  if (myFeed.publish(data)) {
    Serial.println("Data published to MQTT");
  } else {
    Serial.println("Failed to publish data");
  }
}

void start_lap_message(){
  //First message once ESP32 is fully connected
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["team_name"] = "DIOS_TE_AMA";
  jsonDoc["id"] = "$ID_EQUIPO";
  jsonDoc["action"] = "START_LAP";
  // Serialize JSON to chain
  char message[256];
  serializeJson(jsonDoc, message);
  myFeed.publish(topic, message);
}
void end_lap_message(unsigned long total_time){
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["team_name"] = "DIOS_TE_AMA";
  jsonDoc["id"] = "$ID_EQUIPO";
  jsonDoc["action"] = "END_LAP";
  jsonDoc["time"] = total_time;
  // Serialize JSON to chain
  char message[256];
  serializeJson(jsonDoc, message);
  myFeed.publish(topic, message);
}

//void obstacle_message(){
//
//}

void track_loose_message(){
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["team_name"] = "DIOS_TE_AMA";
  jsonDoc["id"] = "$ID_EQUIPO";
  jsonDoc["action"] = "LINE_LOST";
  // Serialize JSON to chain
  char message[256];
  serializeJson(jsonDoc, message);
  myFeed.publish(topic, message);
}

void ping_message(unsigned long lastPingTime){
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["team_name"] = "DIOS_TE_AMA";
  jsonDoc["id"] = "$ID_EQUIPO";
  jsonDoc["action"] = "PING";
  jsonDoc["time"] = lastPingTime;
  // Serialize JSON to chain
  char message[256];
  serializeJson(jsonDoc, message);
  myFeed.publish(topic, message);
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  connectToMQTT();
  //message when starting the lap
  start_lap_message();
  //take initial time:
  TimeStart = millis();
}

void loop() {
  // Mensaje a enviar
  unsigned long partial_time = millis() - lastPingTime;
  //send each 4 seconds
  if (tiempoTranscurrido >= 4000) {
    ping_message(lastPingTime);
    lastPingTime = millis();  // Actualiza el tiempo del último mensaje PING
  }
  //ultrasonidos esté cerca:
  unsigned long total_time = millis() - TimeStart;
  end_lap_message(total_time);
}

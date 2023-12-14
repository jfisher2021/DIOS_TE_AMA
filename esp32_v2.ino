#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>

const char* ssid = "wifieif2";
const char* password = "Goox0sie_WZCGGh25680000";

#define MQTT_SERVER     "garceta.tsc.urjc.es"
#define MQTT_PORT       21883
#define MQTT_USERNAME   "Dios_te_ama"
#define MQTT_KEY        "33"

const char* topic = "/SETR/2023/10/";
const char* id_equipo = "10";

unsigned long TimeStart = 0;
unsigned long lastPingTime = 0; 


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);


void connectToWiFi() {
  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  //Serial.print("RRSI: ");
  //Serial.println(WiFi.RSSI());
}

void connectToMQTT() {
  Serial.println("Connecting to MQTT...");

  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.println("MQTT Connected!");
}

// Publish data to MQTT feed
void publishData(const char* data) {
  if (mqtt.publish(topic,data)) {
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
  publishData(message);
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
  publishData(message);
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
  publishData(message);
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
  publishData(message);
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
  if (!mqtt.connected()) {
    connectToMQTT();
  }
  unsigned long partial_time = millis() - lastPingTime;
  //send each 4 seconds
  if (partial_time >= 4000) {
    ping_message(lastPingTime);
    lastPingTime = millis();  // Actualiza el tiempo del último mensaje PING
  }
  //ultrasonidos esté cerca:
  unsigned long total_time = millis() - TimeStart;

  end_lap_message(total_time);
  exit(EXIT_SUCCESS);
}

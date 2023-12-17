#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char* ssid = "eduroam";
const char* password = "Goox0sie_WZCGGh25680000";

const char* topic = "/SETR/2023/10/";
const char* id_equipo = "10";

unsigned long TimeStart = 0;
unsigned long lastPingTime = 0;

#define RXD2 33
#define TXD2 4

#define MQTT_SERVER "garceta.tsc.urjc.es"
#define MQTT_USERNAME "Dios_te_ama"
#define MQTT_KEY "33"

uint16_t MQTT_PORT = 21883;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  Serial.print("Wifi connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
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
  Serial2.print("{CONNECTED}");
  Serial.println("MQTT Connected!");
}

void publishData(const char* data) {
  if (mqtt.publish(topic, data)) {
    Serial.println("Data published to MQTT");
  } else {
    Serial.println("Failed to publish data");
  }
}

void start_lap_message() {
  char message[256];
  sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"START_LAP\"}", id_equipo);
  publishData(message);
}

void end_lap_message(unsigned long total_time) {
  char message[256];
  sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"END_LAP\",\"time\":%lu}", id_equipo, total_time);
  publishData(message);
}

void track_loose_message() {
  char message[256];
  sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"LINE_LOST\"}", id_equipo);
  publishData(message);
}

void obstacle_detection_message() {
  char message[256];
  sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"OBSTACLE_DETECTED\"}", id_equipo);
  publishData(message);
}

void ping_message(unsigned long lastPingTime) {
  char message[256];
  sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"PING\",\"time\":%lu}", id_equipo, lastPingTime);
  publishData(message);
}

void setup() {
  Serial.begin(9600);
  //Ardiuino
  Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
}

String sendBuff;

void loop() {
  //able to communicate with arduino
  if (Serial2.available()) {

    //read from arduino
    char c = Serial2.read();
    sendBuff += c;

    if (c == '}'){

      //connect to wifi
      connectToWiFi();
      //connect to mqtt
      if (!mqtt.connected()) {
        connectToMQTT();
      }
      
      if (sendBuff == "{CONNECT}"){
        connectToMQTT();
      } else if(sendBuff == "{START_LAP}"){
        start_lap_message();

      } else if (sendBuff == "{LINE_LOST}"){
        track_loose_message();

      } else if (sendBuff == "{OBSTACLE_DETECTED}"){
        obstacle_detection_message();

      } else if(sendBuff == "{PING}"){  
        ping_message();

      } else if(sendBuff == "{END_LAP}"){
        end_lap_message();
      }
      sendBuff="";

      //if (partial_time >= 4000) {
      //  ping_message(lastPingTime);
      //  lastPingTime = millis();
      //}


      //unsigned long total_time = millis() - TimeStart;
      //end_lap_message(total_time);
      // exit(EXIT_SUCCESS); // Commented out as Arduino sketches don't typically exit
    }
  }
}
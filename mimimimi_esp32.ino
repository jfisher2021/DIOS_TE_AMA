#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char* ssid = "LosPollosHermanos";
const char* password = "tdko9519";

const char* topic = "/SETR/2023/10/";
const char* id_equipo = "10";

unsigned long TimeStart = 0;
unsigned long lastPingTime = 0;
unsigned long partial_time = 0;

#define RXD2 33
#define TXD2 4

#define MQTT_SERVER "garceta.tsc.urjc.es"
#define MQTT_USERNAME "Dios_te_ama"
#define MQTT_KEY "33"

uint16_t MQTT_PORT = 21883;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);


unsigned long start_time = millis();
int dist;


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
  sprintf(message, "{\n\t\"team_name\": \"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"START_LAP\"\n}", id_equipo);
  publishData(message);
}

void end_lap_message(unsigned long time) {
  char message[256];
  sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"END_LAP\",\n\t\"time\": %ld\n}", id_equipo,time);
  publishData(message);
}

void track_loose_message() {
  char message[256];
  sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"LINE_LOST\"\n}", id_equipo);
  publishData(message);
}

void obstacle_detection_message() {
  char message[256];
  sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"OBSTACLE_DETECTED\"\"distance\": %d/n}", id_equipo,dist);
  //sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"OBSTACLE_DETECTED\"\n}", id_equipo);
  publishData(message);
}

void ping_message(unsigned long time) {
  char message[256];
  sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"PING\",\n\t\"time\": %ld\n}", id_equipo,time);
  publishData(message);
}

void setup() {
  Serial.begin(9600);
  //Ardiuino
  Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
}

String receive_buff;

void loop() {
  //able to communicate with arduino
  if (Serial2.available()) {

    //read from arduino
    char c = Serial2.read();
    receive_buff += c;

    if (c == '}'){

      //connect to wifi
      connectToWiFi();
      //connect to mqtt
      if (!mqtt.connected()) {
        connectToMQTT();
      }
      if (receive_buff == "{CONNECT}"){
        connectToMQTT();
      } else if(receive_buff == "{START_LAP}"){
        start_time = millis();
        //time for pinging
        partial_time= millis();
        start_lap_message();

      } else if (receive_buff == "{LINE_LOST}"){
        track_loose_message();

      } else if (receive_buff == "{OBSTACLE_DETECTED}"){
        obstacle_detection_message();

      } else if(receive_buff == "{PING}"){  
        ping_message(millis()-start_time);

      } else if(receive_buff == "{END_LAP}"){
        end_lap_message(millis()-start_time);
      }
      receive_buff="";

    }else{
      dist = receive_buff.toInt();
    }
    //char c = Serial2.readStringUntil('\n');
    
    //else if (c== '.'){
    //   String strSinPunto = "";
    //  for (int i = 0; i < receive_buff.length(); i++) {
    //    char caracter = receive_buff.charAt(i);
    //    if (caracter != '.') {
    //      strSinPunto += caracter;
    //    }
    //  }
    //  distance = strSinPunto.toInt();
    //}
  }
}

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

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());
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



void setup() {
  Serial.begin(115200);
  connectToWiFi();
}

void loop() {
  connectToMQTT();
  // Mensaje a enviar

  String mensaje = "Hola, este es un mensaje desde el equipo " + String(id_equipo);

  // Publicar mensaje en el tema específico
  myFeed.publish(topic, mensaje.c_str());
  //publishData("Hello, MQTT!");
}

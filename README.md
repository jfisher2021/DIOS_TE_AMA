# DIOS_TE_AMA

No creo q por cambiar el readme de error
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID       "LosPollosHermanos"
#define WIFI_PASS       "tdko9519"

#define MQTT_SERVER     "io.adafruit.com"
#define MQTT_PORT       1883
#define MQTT_USERNAME   "Dios_te_ama"
#define MQTT_KEY        "33"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);

Adafruit_MQTT_Publish   myFeed = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "/feeds/myfeed");


void connectToWiFi() {
  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
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
  publishData("Hello, MQTT!");
}

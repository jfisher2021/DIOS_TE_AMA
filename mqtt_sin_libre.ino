#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "credentials.h"

const char* ssid = "eduroam";

const char* topic = "/SETR/2023/10/";
const char* id_equipo = "10";

unsigned long TimeStart = 0;
unsigned long lastPingTime = 0;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);

void connectToWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
    Serial.print("Connecting to WiFi ..");
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

void ping_message(unsigned long lastPingTime) {
    char message[256];
    sprintf(message, "{\"team_name\":\"DIOS_TE_AMA\",\"id\":\"%s\",\"action\":\"PING\",\"time\":%lu}", id_equipo, lastPingTime);
    publishData(message);
}

void setup() {
    Serial.begin(115200);
    connectToWiFi();
    connectToMQTT();
    start_lap_message();
    TimeStart = millis();
}

void loop() {
    if (!mqtt.connected()) {
        connectToMQTT();
    }
    unsigned long partial_time = millis() - lastPingTime;
    if (partial_time >= 4000) {
        ping_message(lastPingTime);
        lastPingTime = millis();
    }

    unsigned long total_time = millis() - TimeStart;
    end_lap_message(total_time);
    // exit(EXIT_SUCCESS); // Commented out as Arduino sketches don't typically exit
}

#include "WiFi.h"

const char* ssid = "wifieif";
const char* password = "Goox0sie_WZCGGh25680000";

void initWiFi() {

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

void setup() {

  Serial.begin(115200);
  initWiFi();
  
}

void loop() {
}

#include "FastLED.h"
#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  // Set Red Color to LED
  FastLED.showColor(Color(255, 0, 0));
  delay(1000);

  String sendBuff;
  Serial.println("{CONNECT}");

  // To make this code work, remember that the switch S1 should be set to "CAM"
  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      sendBuff += c;
      if (c == '}') {
        Serial.print("Received data in serial port from ESP32: ");
        Serial.println(sendBuff);

        // Set Red Green to LED
        FastLED.showColor(Color(0, 255, 0));
        sendBuff = "";
        delay(1000);
        break;
      }
    }
  }
}

void loop() {
  Serial.println("{START_LAP}");
  delay(1000); // Espera 1 segundo antes de enviar el próximo mensaje
}

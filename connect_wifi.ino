#define RXD2 33
#define TXD2 4

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(5000);
  Serial2.print("{ 'test': " + String(millis()) + " }");
  Serial.print("Messase sent! to Arduino");
}

String sendBuff;

void loop() {
  if (Serial2.available()) {
    char c = Serial2.read();
    sendBuff += c;

    if (c == '}') {
      Serial.print("Received data in serial port from Arduino: ");
      Serial.println(sendBuff);

      // Procesa el mensaje recibido
      if (sendBuff == "{CONNECT}") {


        Serial2.print("{CONNECTED}");
        Serial.println("{CONNECTED}");
      } else if (sendBuff == "{START_LAP}") {


        Serial2.print("{START_LAP}");
        Serial.println("{START_LAP}");
      }

      // Limpia el buffer para el próximo mensaje
      sendBuff = "";
    }
  }
}

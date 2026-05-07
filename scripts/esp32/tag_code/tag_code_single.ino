#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

HardwareSerial MySerial(2);  // Use UART2 (pins 16=RX, 17=TX by default)
const char* ssid = "iptime11";
const char* password = "korea123#";

const char* jetson_ip = "192.168.0.100";
const int jetson_port = 5005;
String line = "";

WiFiUDP udp;

char message[64];

void talk(String msg) {
  MySerial.write(msg.c_str());
  if (MySerial.available() > 0) {
    String data = MySerial.readString();
    Serial.println(data);
  }
}

void send(String msg) {
  MySerial.write(msg.c_str());
}

void readTag() {
  if (MySerial.available() > 0) {
    String data = MySerial.readString();
    Serial.println(data);
  }
}

void setup() {
  Serial.begin(115200);     
  MySerial.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  Serial.println("ESP32 started, listening on Serial2 (pins 16=RX, 17=TX)");
  delay(100);
  talk("AT+anchor_tag=0,20\r\n");
  delay(5000);
  talk("AT+RST\r\n");
  delay(5000);
  talk("AT+interval=50\r\n");
  delay(5000);
  talk("AT+switchdis=1\r\n");
  delay(5000);
}

void loop() {
  while (MySerial.available()) {
    char c = MySerial.read();
    if (c == '\n') {
      // full message received
      udp.beginPacket(jetson_ip, jetson_port);
      udp.write((const uint8_t*)line.c_str(), line.length());
      udp.endPacket();
      Serial.println("Sent: " + line);
      line = "";                    // clear for next line
    } else {
      line += c;
      if (line.length() > 512) line = "";  // safety overflow guard
    }
  }
}

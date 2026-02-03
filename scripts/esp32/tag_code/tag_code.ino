#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define DISTANCES_PER_ANCHOR_PER_CYCLE 6
#define NUMBER_OF_ANCHORS 4
#define DELAY_TIME_MS 28

HardwareSerial MySerial(2);
const char* ssid = "iptime11";
const char* password = "korea123#";

const char* jetson_ip = "192.168.0.100";
const int jetson_port = 5005;
String line = "";

const int esp_listen_port = 5006;

unsigned char count_sent_fixes = 0;
String TAG_ADDRESS = "20";

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

String parseCmd(const String cmd) {
  int dash_idx = cmd.indexOf('-');
  int colon_idx = cmd.indexOf(':');
  if (dash_idx < 0 || colon_idx < 0 || colon_idx <= dash_idx) return "error";

  int end_idx = cmd.indexOf('\r', colon_idx + 1);
  if (end_idx < 0) end_idx = cmd.indexOf('\n', colon_idx + 1);
  if (end_idx < 0) end_idx = cmd.length();

  String addr = cmd.substring(dash_idx + 1, colon_idx);      // "10"/"20"
  String verb = cmd.substring(colon_idx + 1, end_idx);       // "START"/"STOP"
  verb.trim();                                               // remove whitespace

  if (verb == "START") return "ST:" + addr;
  if (verb == "STOP")  return "SP:" + addr;
  return "error";
}

void setup() {
  Serial.begin(115200);     
  MySerial.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);

  unsigned long start_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    if (millis() - start_time > 10000) {
      Serial.println("\nWiFi connection failed!");
      return;
    }
  }
  Serial.println("\nWiFi Connected!");

  Serial.println("ESP32 started, listening on Serial2 (pins 16=RX, 17=TX)");
  delay(100);
  talk("AT+anchor_tag=0," + TAG_ADDRESS + "\r\n");
  delay(5000);
  talk("AT+RST\r\n");
  delay(5000);
  talk("AT+interval=5\r\n");
  delay(5000);
  if (TAG_ADDRESS.equals("10")) {
    talk("AT+switchdis=1\r\n");
  } else if (TAG_ADDRESS.equals("20")) {
    talk("AT+switchdis=0\r\n");
  }
  delay(5000);
  udp.begin(esp_listen_port);

  delay(DELAY_TIME_MS);

  udp.beginPacket(jetson_ip, jetson_port);
  line = "ADDRESS-" + TAG_ADDRESS + ":HELLO\r\n";
  udp.write((const uint8_t*)line.c_str(), line.length());
  udp.endPacket();
  Serial.println("Sent: " + line);
  line = "";
}

void loop() {
  while (MySerial.available()) {
    char c = MySerial.read();
    if (c == '\n') {
      udp.beginPacket(jetson_ip, jetson_port);
      udp.write((const uint8_t*)line.c_str(), line.length());
      udp.endPacket();
      Serial.println("Sent: " + line);
      line = "";
      count_sent_fixes++;
    } else {
      line += c;
      if (line.length() > 512) {
        Serial.println("Warning: Line exceeded max length, discarding message");
        line = "";
      }
    }
  }
  if (count_sent_fixes >= DISTANCES_PER_ANCHOR_PER_CYCLE * NUMBER_OF_ANCHORS) {
    count_sent_fixes = 0;
    talk("AT+switchdis=0\r\n");
    delay(DELAY_TIME_MS);
    talk("AT+RST\r\n");
    delay(DELAY_TIME_MS);
    line = "ADDRESS-" + TAG_ADDRESS + ":FINISHED\r\n";
    udp.beginPacket(jetson_ip, jetson_port);
    udp.write((const uint8_t*)line.c_str(), line.length());
    udp.endPacket();
    Serial.println("Sent: " + line);
    line = "";
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    if (udp.parsePacket() > 0) {
      while (udp.available()) {
        char udp_char = udp.read();
        line += udp_char;
        
        if (udp_char == '\n') {
          String jetson_cmd = parseCmd(line);
          if (jetson_cmd.startsWith("ST") && TAG_ADDRESS.equals(jetson_cmd.substring(jetson_cmd.length() - 2))) {
            talk("AT+anchor_tag=0," + TAG_ADDRESS + "\r\n");
            delay(DELAY_TIME_MS);
            talk("AT+RST\r\n");
            delay(DELAY_TIME_MS);
            talk("AT+switchdis=1\r\n");
            delay(DELAY_TIME_MS);
            break;
          } else if (jetson_cmd.startsWith("SP") && TAG_ADDRESS.equals(jetson_cmd.substring(jetson_cmd.length() - 2))) {
            talk("AT+switchdis=0\r\n");
            delay(DELAY_TIME_MS);
            talk("AT+RST\r\n");
            delay(DELAY_TIME_MS);
            break;
          } else {
            Serial.println("ERROR: Wrong command or address\r");
          }
          line = "";
        }

        if (line.length() > 512) {
          Serial.println("WARNING: Line exceeded max length, discarding message\r");
          line = "";
        }
      }
    }
  }
}

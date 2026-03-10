#include <HardwareSerial.h>
#include <WiFi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <errno.h>
#include <string.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DISTANCES_PER_ANCHOR_PER_CYCLE 5
#define NUMBER_OF_ANCHORS 4
#define DELAY_TIME_MS 80
#define BUF_SIZE 512

HardwareSerial MySerial(2);
Adafruit_MPU6050 mpu;

const char* ssid = "iptime11";
const char* password = "korea123#";

const char* jetson_ip = "192.168.0.100";
const int jetson_port = 5005;
const int esp_listen_port = 5006;

static TaskHandle_t urgent_handle = NULL;

// Ring buffer handle must be global (shared by tasks)
static RingbufHandle_t cmd_rb = NULL;

static int sock_listener = -1;
static int sock_sender   = -1;

static struct sockaddr_in addr_listen;
static struct sockaddr_in addr_send;

static unsigned char count_sent_fixes = 0;
static String TAG_ADDRESS = "20";
static String imu_msg;

// --------- Helpers ---------

void talk(const String &msg) {
  MySerial.write((const uint8_t*)msg.c_str(), msg.length());
  // Optional read-back (as you had)
  if (MySerial.available() > 0) {
    String data = MySerial.readString();
    //Serial.println(data);
  }
}

String parseCmd(const String cmd) {
  int dash_idx = cmd.indexOf('-');
  int colon_idx = cmd.indexOf(':');
  if (dash_idx < 0 || colon_idx < 0 || colon_idx <= dash_idx) return "error";

  int end_idx = cmd.indexOf('\r', colon_idx + 1);
  if (end_idx < 0) end_idx = cmd.indexOf('\n', colon_idx + 1);
  if (end_idx < 0) end_idx = cmd.length();

  String addr = cmd.substring(dash_idx + 1, colon_idx); // "10"/"20"
  String verb = cmd.substring(colon_idx + 1, end_idx);  // "START"/"STOP"
  verb.trim();

  if (verb == "START") return "ST:" + addr;
  if (verb == "STOP")  return "SP:" + addr;
  return "error";
}

// Send UDP to Jetson (safe from any task)
static void udp_send_line(const char *data, size_t len) {
  if (sock_sender < 0) return;
  int rc = sendto(sock_sender, data, len, 0, (struct sockaddr*)&addr_send, sizeof(addr_send));
  if (rc < 0) {
    Serial.printf("[UDP SEND ERROR] errno=%d (%s)\n", errno, strerror(errno));
  }
}

// --------- Tasks ---------

// Task 1: Read Serial2 line-by-line, forward to Jetson. Also FINISHED logic (matches your bottom code).
static void serial_forward_task(void *arg) {
  String line;
  line.reserve(BUF_SIZE);

  for (;;) {
    while (MySerial.available()) {
      char c = (char)MySerial.read();
      line += c;

      if (line.length() > BUF_SIZE) {
        Serial.println("Warning: Line exceeded max length, discarding message");
        line = "";
        continue;
      }

      if (c == '\n') {
        udp_send_line(line.c_str(), line.length());
        Serial.print("Sent: ");
        Serial.print(line);

        line = "";
        count_sent_fixes++;

        if (count_sent_fixes >= DISTANCES_PER_ANCHOR_PER_CYCLE * NUMBER_OF_ANCHORS) {
          count_sent_fixes = 0;

          talk("AT+switchdis=0\r\n");
          vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));
          talk("AT+RST\r\n");
          vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));

          String finished = "ADDRESS-" + TAG_ADDRESS + ":FINISHED\r\n";
          udp_send_line(finished.c_str(), finished.length());
          Serial.print("Sent: ");
          Serial.print(finished);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Task 2: Receive UDP packets (Jetson commands). Push entire packet into ring buffer, notify urgent task.
static void udp_rx_task(void *arg) {
  uint8_t _buf[BUF_SIZE];

  for (;;) {
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    int n = recvfrom(sock_listener, _buf, sizeof(_buf) - 1, 0,
                     (struct sockaddr*)&from_addr, &from_len);

    if (n < 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    _buf[n] = '\0';
    size_t item_len = (size_t)n + 1;

    // Acquire a contiguous space from ring buffer
    void *item = NULL;
    BaseType_t ok = xRingbufferSendAcquire(cmd_rb, &item, item_len, pdMS_TO_TICKS(50));
    if (ok != pdTRUE || item == NULL) {
      Serial.println("[WARN] Ring buffer full, dropping UDP cmd");
      continue;
    }

    memcpy(item, _buf, item_len);

    if (xRingbufferSendComplete(cmd_rb, item) != pdTRUE) {
      Serial.println("[ERROR] xRingbufferSendComplete failed");
      // nothing else to do here
      continue;
    }

    if (urgent_handle) xTaskNotifyGive(urgent_handle);
  }
}

// Task 3: Urgent command handler (START/STOP). Woken via notify.
static void urgent_task(void *arg) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Drain all pending commands quickly
    for (;;) {
      size_t item_size = 0;
      uint8_t* item = (uint8_t*)xRingbufferReceive(cmd_rb, &item_size, 0);
      if (!item) break;

      // item is null-terminated as stored
      String line = String((const char*)item);
      vRingbufferReturnItem(cmd_rb, (void*)item);  // IMPORTANT: free ring buffer item

      String jetson_cmd = parseCmd(line);

      if (jetson_cmd.startsWith("ST") &&
          TAG_ADDRESS.equals(jetson_cmd.substring(jetson_cmd.length() - 2))) {
        talk("AT+anchor_tag=0," + TAG_ADDRESS + "\r\n");
        vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));
        talk("AT+RST\r\n");
        vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));
        talk("AT+switchdis=1\r\n");
        vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));

      } else if (jetson_cmd.startsWith("SP") &&
                 TAG_ADDRESS.equals(jetson_cmd.substring(jetson_cmd.length() - 2))) {

        talk("AT+switchdis=0\r\n");
        vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));
        talk("AT+RST\r\n");
        vTaskDelay(pdMS_TO_TICKS(DELAY_TIME_MS));

      } else {
        Serial.print("ERROR: Wrong command or address: ");
        //Serial.println(line);
      }
    }
  }
}

// --------- Socket setup ---------

static bool setup_sockets() {
  // Listener socket
  sock_listener = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock_listener < 0) {
    Serial.printf("[ERROR] socket(listener) failed errno=%d\n", errno);
    return false;
  }

  int yes = 1;
  setsockopt(sock_listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  memset(&addr_listen, 0, sizeof(addr_listen));
  addr_listen.sin_family = AF_INET;
  addr_listen.sin_port = htons(esp_listen_port);
  addr_listen.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock_listener, (struct sockaddr*)&addr_listen, sizeof(addr_listen)) < 0) {
    Serial.printf("[ERROR] bind(listener) failed errno=%d (%s)\n", errno, strerror(errno));
    close(sock_listener);
    sock_listener = -1;
    return false;
  }

  // Sender socket
  sock_sender = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock_sender < 0) {
    Serial.printf("[ERROR] socket(sender) failed errno=%d\n", errno);
    close(sock_listener);
    sock_listener = -1;
    return false;
  }

  // Destination = Jetson
  memset(&addr_send, 0, sizeof(addr_send));
  addr_send.sin_family = AF_INET;
  addr_send.sin_port = htons(jetson_port);
  addr_send.sin_addr.s_addr = inet_addr(jetson_ip); // <-- critical fix

  return true;
}

// --------- Arduino entry points ---------

void setup() {
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, 16, 17);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

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

  if (!setup_sockets()) {
    Serial.println("[FATAL] Socket setup failed");
    return;
  }

  // Ring buffer: store a few commands. Make it bigger than BUF_SIZE to allow multiple packets queued.
  cmd_rb = xRingbufferCreate(4 * BUF_SIZE, RINGBUF_TYPE_NOSPLIT);
  if (!cmd_rb) {
    Serial.println("[FATAL] Failed to create ring buffer");
    return;
  }

  // Initial AT init (same as your bottom code)
  Serial.println("ESP32 started, listening on Serial2 (pins 16=RX, 17=TX)");
  delay(100);
  talk("AT+anchor_tag=0," + TAG_ADDRESS + "\r\n");
  delay(5000);
  talk("AT+RST\r\n");
  delay(5000);
  talk("AT+interval=5\r\n");
  delay(5000);

  if (TAG_ADDRESS.equals("10")) talk("AT+switchdis=1\r\n");
  else if (TAG_ADDRESS.equals("20")) talk("AT+switchdis=0\r\n");
  delay(100);

  // Send HELLO (same as bottom code)
  String hello = "ADDRESS-" + TAG_ADDRESS + ":HELLO\r\n";
  udp_send_line(hello.c_str(), hello.length());
  Serial.print("Sent: ");
  Serial.print(hello);

  // Start tasks
  xTaskCreatePinnedToCore(urgent_task, "urgent", 4096, NULL, 10, &urgent_handle, 0);
  xTaskCreatePinnedToCore(udp_rx_task, "udp_rx", 4096, NULL, 8, NULL, 0);
  xTaskCreatePinnedToCore(serial_forward_task, "serial_fwd", 4096, NULL, 6, NULL, 1);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  imu_msg = "ADDRESS:" + TAG_ADDRESS + " ACC:" + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z) + " GYRO:" + String(a.gyro.x) + "," + String(a.gyro.y) + "," + String(a.gyro.z) + "\r\n";
  udp_send_line(imu_msg.c_str(), imu_msg.length());
  vTaskDelay(pdMS_TO_TICKS(10));
}

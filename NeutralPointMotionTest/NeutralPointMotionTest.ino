#include <Wire.h>
#include <MS5837.h>
#include <esp_timer.h>
#include <LittleFS.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <RTClib.h>

#define DIR_PIN 16
#define PWM_PIN 17
#define MOTOR_SPEED 255

#define POT_PIN 34
#define POT_LOWER_LIMIT (800 + 50)
#define POT_UPPER_LIMIT (3150 - 50)
#define DIR_UP LOW
#define DIR_DOWN HIGH

#define FLUID_DENSITY 997
#define SET_POINT 2.5f
#define HYSTERESIS 0.3f
#define RANGE 0.5f

#define DEPTH_UPPER_LIMIT (SET_POINT - RANGE)
#define DEPTH_LOWER_LIMIT (SET_POINT + RANGE)

hw_timer_t *timer = NULL;
File file;
esp_now_peer_info_t peer;
RTC_DS3231 rtc;

volatile float initialDepth = 0;
volatile float depth = 0;
int lastDepthAddr = 0;

MS5837 sensor;
unsigned long long trialTime = 0;
unsigned long long lastReadingTime = 0;
unsigned long long lastTime;
unsigned long long timer2;
bool timerEnded = false;
// bool inRange = false;
bool sendingData = false;

float x = 0;

int readPot() {
  x = ( 99.0 * x + (analogRead(POT_PIN)/4)) / 100.0;
  float y = ( x * 4095.0 /1134.0);
  return y;
}

void formatLittleFS() {
  Serial.println("Formatting LittleFS...");
  if (LittleFS.format()) {
    Serial.println("LittleFS formatted successfully.");
  } else {
    Serial.println("Failed to format LittleFS.");
  }
}

void espNowSend(String data) {
  stopMotor();
  int index = 0;
  while (index < data.length()) {
    int sentSize = min(data.length() - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
    ESP_ERROR_CHECK(esp_now_send(peer.peer_addr, (uint8_t*)(data.c_str() + index), sentSize));
    index += sentSize;
  }
}

void sendDepthData() {
  static bool fileOpen = false;
  if(sendingData){
    if (!fileOpen){
    file.close();
    file = LittleFS.open("/data.txt", FILE_READ);
    if (!file) {
      Serial.println("Failed to open file for Reading");
      return;
    }
    fileOpen = true;
    }
    while (file.available()) {
      espNowSend(file.readStringUntil('\n'));
      delay(35);
    } 
  } else {
    fileOpen = false;
    file.close();
    file = LittleFS.open("/data.txt", FILE_APPEND);
  }
  espNowSend("finished sending");
}

void initRTC() {
    if (! rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }
}

void initEspNow() {
  stopMotor();
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("esp32", "esp32");
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_now_init());

  // set mac address of the peer
  // cc:7b:5c:a7:7f:cc
  uint8_t peer_address[] = { 0xcc, 0x7b, 0x5c, 0xa7, 0x7f, 0xcc };
  for (int i = 0; i < 6; i++) {
    peer.peer_addr[i] = peer_address[i];
  }

  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void initLittleFS() {
  stopMotor();
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    formatLittleFS();  // Format LittleFS if mount fails
    if (!LittleFS.begin()) {
      Serial.println("LittleFS Mount Failed after formatting");
      return;
    }
  }
  file = LittleFS.open("/data.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for Reading");
  }

  // while(file.available()) {
  //   Serial.println(file.readStringUntil('\n'));
  // }

  file.close();
  file = LittleFS.open("/data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.printf("Modified Hysterisis Run\n\n");
  file.flush();
}

void updateDepth() {
  sensor.read();
  depth = sensor.depth() - initialDepth + 0.335;
  // file.printf("Timestamp: %s Depth: %f PotPosition: %d\n", getRTCTime().c_str(), depth, readPot());
  file.flush(); // Ensure data is written
  // Serial.printf("Timestamp: %s Depth: %f PotPosition: %d\n", getRTCTime().c_str(), depth, readPot());
  lastReadingTime = millis();
}

void initBar30() {
  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(FLUID_DENSITY); // kg/m^3 (freshwater, 1029 for seawater)
  sensor.read();
  initialDepth = abs(sensor.depth());
  Serial.print("initial Depth = ");
  Serial.println(initialDepth);
}

void initPins() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
}

// String getRTCTime() {
//   DateTime now = rtc.now();
//   String dateTimeString = String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
//   return dateTimeString;
// }

void moveMotorUp() {
  digitalWrite(DIR_PIN, DIR_UP);
  analogWrite(PWM_PIN, MOTOR_SPEED);
}

void moveMotorDown() {
  digitalWrite(DIR_PIN, DIR_DOWN);
  analogWrite(PWM_PIN, MOTOR_SPEED);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
}

void setup() {
  Serial.begin(115200);
  initPins();
  Serial.println("Pins intialized");
  initLittleFS();
  Serial.println("LittleFS intialized");
  initEspNow();
  Serial.println("EspNOW intialized");
  initBar30();
  Serial.println("Bar30 intialized");
  // initRTC();
  // Serial.println("RTC Initialized");

  // sendingData = true;
  // for(int i = 0; i<=10; i++ ){
  // sendDepthData();
  // }
  // sendingData = false;
  // sendDepthData();

  // go up
  while (readPot() <= POT_UPPER_LIMIT) {
    moveMotorUp();
  }
  stopMotor();

  lastTime = micros(); // to account for setup time.
  timer2 = millis();
  Serial.println("Loop Starting");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();
}

bool inRange() {
  return depth > DEPTH_UPPER_LIMIT && depth < DEPTH_LOWER_LIMIT;
}

bool shouldGoUp() {
  return depth > SET_POINT + HYSTERESIS && readPot() <= POT_UPPER_LIMIT;
}

bool shouldGoDown() {
  return depth < SET_POINT - HYSTERESIS && readPot() >= POT_LOWER_LIMIT;
}

bool inHisteresis() {
  return depth < SET_POINT + HYSTERESIS && depth > SET_POINT - HYSTERESIS;
}

void loop() {
  updateDepth();
  ArduinoOTA.handle();

  if (!timerEnded) {
    if (trialTime >= 45 * 1000 * 1000) {
      timerEnded = true;
      return;
    }

    int neutralPoint = 1000;
    // 0 -> POT_LOWER_LIMIT (120)
    // 2.5 -> neutral point

    // 2.5 -> neutral point
    // 5 -> POT_UPPER_LIMIT (3200)

    int requiredPotPosition = 0;
    if (depth > 2.5f)
      requiredPotPosition = (depth - 2.5f) / 2.5f * (POT_UPPER_LIMIT - neutralPoint) + neutralPoint;
    else
      requiredPotPosition = depth / 2.5f * (neutralPoint - POT_LOWER_LIMIT) + POT_LOWER_LIMIT;

    if (readPot() > requiredPotPosition + 50) {
      moveMotorDown();
      // Serial.println("moving down");
    } else if (readPot() < requiredPotPosition - 50) {
      moveMotorUp();
      // Serial.println("moving up");
    } else {
      stopMotor();
      // Serial.println("stop moving ");
    }

    if (inRange()) {
      // in range
      // espNowSend("in range");
      // Serial.println("in range");
      trialTime += micros() - lastTime;
    }
    
    lastTime = micros();
  } else {
    // timer finished
    // espNowSend("timer finished");
    if (readPot() <= POT_UPPER_LIMIT) {
      moveMotorUp();
    } else {
      stopMotor();
      sendingData = true;
      sendDepthData();
    }
  }
}

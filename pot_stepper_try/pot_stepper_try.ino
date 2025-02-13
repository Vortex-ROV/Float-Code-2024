#include <Wire.h>
#include <MS5837.h>
#include <esp_timer.h>
#include <LittleFS.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string>
#include <ESPmDNS.h>
// #include <NetworkUdp.h>
#include <ArduinoOTA.h>
// #include <RTClib.h>

// #define DIR_PIN 16
#define PWM_PIN 17
#define MOTOR_SPEED 255

#define DIR_PIN 32
#define STEP_PIN 33
#define DIR_UP HIGH
#define DIR_DOWN LOW
#define ENABLE_PIN 23
#define STEPPER_STEP_TIME 70
#define STEPPER_TIMER_TIME 100

#define POT_PIN 34
#define POT_LOWER_LIMIT (200 + 100)
#define POT_UPPER_LIMIT (2800 - 100)
// #define DIR_UP LOW
// #define DIR_DOWN HIGH


#define FLUID_DENSITY 997
#define SET_POINT 2.5f
#define HYSTERESIS 0.3f
#define RANGE 0.5f

#define DEPTH_UPPER_LIMIT (SET_POINT - RANGE)
#define DEPTH_LOWER_LIMIT (SET_POINT + RANGE)


#define DEPTH_HISTORY_SIZE 2000
float depthHistory[DEPTH_HISTORY_SIZE];
int depthIndex = 0;

hw_timer_t *timer = NULL;
File file;
esp_now_peer_info_t peer;
// RTC_DS3231 rtc;

volatile float initialDepth = 0;
volatile float depth = 0;
 int lastDepthAddr = 0;

bool done = false;
bool stopped = false;

MS5837 sensor;
unsigned long long trialTime = 0;
unsigned long long lastReadingTime = 0;
unsigned long long lastTime;
unsigned long long timer2;
bool timerEnded = false;
// bool inRange = false;
bool sendingData = false;
bool stepperState = false;

float x = 0;

int readPot() {
  x = ( 99.0 * x + (analogRead(POT_PIN)/4)) / 100.0;
  float y = ( x * 4095.0 /1134.0);
  Serial.println(y);
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
  int index = 0;
  while (index < data.length()) {
    int sentSize = min(data.length() - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
    ESP_ERROR_CHECK(esp_now_send(peer.peer_addr, (uint8_t*)(data.c_str() + index), sentSize));
    index += sentSize;
  }
}

void initEspNow() {
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

void updateDepth() {
  if (millis() - lastReadingTime < 15)
    return;
    
  sensor.read();
  depth = sensor.depth() - initialDepth + 0.335;
  
  // Store depth in array
  depthHistory[depthIndex] = depth;
  depthIndex = (depthIndex + 1) % DEPTH_HISTORY_SIZE; // Circular buffer
  file.flush(); // Ensure data is written
  lastReadingTime = millis();
}

float calculateStdDev(float* stdDev, float* mean) {
  float sum = 0, variance = 0;

  // Compute mean
  for (int i = 0; i < DEPTH_HISTORY_SIZE; i++) {
    sum += depthHistory[i];
  }
  *mean = sum / DEPTH_HISTORY_SIZE;

  // Compute variance
  for (int i = 0; i < DEPTH_HISTORY_SIZE; i++) {
    variance += pow(depthHistory[i] - *mean, 2);
  }
  variance /= DEPTH_HISTORY_SIZE;

  // Compute standard deviation
  *stdDev = sqrt(variance);
  return *stdDev;
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
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
}

void moveMotorUp() {
  stopped = false;
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_UP);
  Serial.println("moving motor up");
}

void moveMotorDown() {
  stopped = false;
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_DOWN);
  Serial.println("moving motor down");
}

void stopMotor() {
  stopped = true;
  digitalWrite(ENABLE_PIN, HIGH);
  Serial.println("stopping motor");
}

void IRAM_ATTR onTimer() {
  if (!stopped) {
    stepperState = !stepperState;
    digitalWrite(STEP_PIN, stepperState);
  }
}

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  initPins();
  Serial.println("Pins intialized");
  initEspNow();
  Serial.println("EspNOW intialized");
  initBar30();
  Serial.println("Bar30 intialized");

  timer = timerBegin(1000000);
  if (timer == NULL) {
      Serial.println("Failed to initialize timer");
      return;
  } else {
      Serial.println("Timer intialized");
  }
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, STEPPER_TIMER_TIME, true, 0);

  // go up
  while (readPot() <= POT_UPPER_LIMIT) {
    moveMotorUp();
  }

  previousMillis = millis();

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

const unsigned long interval = 120 * 1000;
void loop() {
  ArduinoOTA.handle();
  unsigned long currentMillis = millis();

  if (!done && currentMillis - previousMillis <= interval) {
    updateDepth();
    int potPosition = 200;

    // 1000 -> sink
    // 1250 -> sink
    // 1600 -> sink
    // 1800 -> sink
    // 2000 -> sink
    // 2475 -> sink
    // 2593 -> good
    // 2712 -> float

    Serial.println(readPot());

    // Adjust motor based on potentiometer position
    if (readPot() > potPosition + 50) {
      // go down
      moveMotorDown();
    } else if (readPot() < potPosition - 50) {
      // go up
      moveMotorUp();
    } else {
      // stop motor
      stopMotor();
    }
  } else {
    while (readPot() <= POT_UPPER_LIMIT) {
      moveMotorUp();
    }
    stopMotor();

    float stdDev, mean;
    calculateStdDev(&stdDev, &mean);

    espNowSend("std dev:");
    espNowSend(String(stdDev));
    espNowSend("mean:");
    espNowSend(String(mean));

    delay(100);
    done = true;
  }
}

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


#define dirPin 32
#define stepPin 33
#define potPin 34
#define potLowerLimit 143
#define potUpperLimit 2900
#define dirUP HIGH
#define dirDown LOW
#define ENABLE_PIN 23
#define stepperStepTime 70
#define StepperTimerTime 100
#define depthUpperLimit 1
#define depthLowerLimit 2

#define FLUID_DENSITY 997
#define SET_POINT 1.5f
#define HYSTERESIS 0.3f

hw_timer_t *timer = NULL;
volatile bool stepperState = HIGH;
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
bool inRange = false;
bool sendingData = false;

void IRAM_ATTR onTimer() {
  stepperState = !stepperState;
  digitalWrite(stepPin, stepperState);
}

int readPot(){
  return analogReadMilliVolts(potPin);
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
  digitalWrite(ENABLE_PIN, HIGH);
  int index = 0;
  while (index < data.length()) {
    int sentSize = min(data.length() - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
    ESP_ERROR_CHECK(esp_now_send(peer.peer_addr, (uint8_t*)(data.c_str() + index), sentSize));
    index += sentSize;
  }
}

void toggleStep(){
  stepperState = !stepperState;
  digitalWrite(stepPin, stepperState);
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
    if (file.available()) {
      espNowSend(file.readStringUntil('\n'));
      delay(35);
  } 
  } else {
    fileOpen = false;
    file.close();
    file = LittleFS.open("/data.txt", FILE_APPEND);
  }
}

void initRTC(){
    if (! rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }
}

void initEspNow() {
  digitalWrite(ENABLE_PIN, HIGH);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_now_init());

  // set mac address of the peer
  // cc:7b:5c:a7:7f:cc
  uint8_t peer_address[] = { 0xd0, 0xef, 0x76, 0x13, 0xb9, 0x18 };
  for (int i = 0; i < 6; i++) {
    peer.peer_addr[i] = peer_address[i];
  }

  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void initLittleFS() {
  digitalWrite(ENABLE_PIN, HIGH);
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

  while(file.available()) {
    Serial.println(file.readStringUntil('\n'));
  }

  file.close();
  file = LittleFS.open("/data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.printf("Modified Hysterisis Run\n\n");
  file.flush();
  digitalWrite(ENABLE_PIN, LOW);
}

void updateDepth(){
  if((millis() - lastReadingTime) >= 500){
  sensor.read();
  depth = sensor.depth() - initialDepth + 0.335;
  file.printf("Timestamp: %s Depth: %f PotPosition: %d\n", getRTCTime().c_str(), depth, readPot());
  file.flush(); // Ensure data is written
  Serial.printf("Timestamp: %s Depth: %f PotPosition: %d\n", getRTCTime().c_str(), depth, readPot());
  lastReadingTime = millis();
  }
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
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}

String getRTCTime() {
  DateTime now = rtc.now();
  String dateTimeString = String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  return dateTimeString;
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
  initRTC();
  Serial.println("RTC Initialized");

  sendingData = true;
  for(int i = 0; i<=10; i++ ){
  sendDepthData();
  }
  sendingData = false;
  sendDepthData();

  // go up
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(dirPin, dirUP);
  while(readPot() <= potUpperLimit){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepperStepTime);
      }

  // Initialize the timer
  timer = timerBegin(1000000);
  if (timer == NULL) {
      Serial.println("Failed to initialize timer");
      return;
  } else {
      Serial.println("Timer intialized");
  }
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer,StepperTimerTime,true, 0);

  lastTime = micros(); // to account for setup time.
  timer2 = millis();
  Serial.println("Loop Starting");
}

void loop() {
  // ------------------- hysteresis control -------------------
    updateDepth();

    if(!timerEnded){
      // go up
      if(trialTime >= 45000000){
        timerEnded = true;
      }
      else if ((depth > SET_POINT + HYSTERESIS) && (readPot() <= potUpperLimit)) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(dirPin, dirUP);
      }
      // go down
      else if ((depth < SET_POINT - HYSTERESIS) && (readPot() >= potLowerLimit)) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(dirPin, dirDown);
      } 
      else if ((depth < SET_POINT + HYSTERESIS) && (depth > SET_POINT - HYSTERESIS)) {
        int potPosition = map(depth*100, (SET_POINT - HYSTERESIS)*100, (SET_POINT + HYSTERESIS)*100, potLowerLimit, potUpperLimit);
        Serial.println(potPosition);
        if(readPot() > potPosition){
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(dirPin, dirDown);
        } else if (readPot() < potPosition){
            digitalWrite(ENABLE_PIN, LOW);
            digitalWrite(dirPin, dirUP);
        } else {
          digitalWrite(ENABLE_PIN, HIGH);
        }
      }
      else{
        digitalWrite(ENABLE_PIN, HIGH);
      }
      // float is in range
      if (depth > depthUpperLimit && depth < depthLowerLimit) {
        trialTime += micros() - lastTime;
      }
      lastTime = micros();
    } else {
      // timer finished
      if (readPot() <= potUpperLimit) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(dirPin, dirUP);
      } else {
        digitalWrite(ENABLE_PIN, HIGH);
        sendingData = true;
        sendDepthData();
      }
    }
  
}


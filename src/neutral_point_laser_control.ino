#if 1

// mac address: CC:7B:5C:A7:7F:CC

#include <Wire.h>
#include "MS5837.h"
#include <esp_timer.h>
#include <LittleFS.h>
#include "VL6180X.h"
#include <ArduinoOTA.h>
#include <RTClib.h>
#include "communication.h"
#include "sensors.h"
#include "stepper.h"
#include "control.h"
#include "filesystem.h"
#include <queue>

hw_timer_t *timer = NULL;

esp_now_peer_info_t espNowPeer;
VL6180X irSensor;
MS5837 bar30;
File file;
RTC_DS3231 rtc;

std::queue<String> q;
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

// lightweight function
void onReceive(const uint8_t *esp_now_info, const uint8_t *data, int data_len) {
  q.emplace(String((char*)data));
}

void initPins() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  initPins();
  Serial.println("Pins intialized");
  initEspNow(espNowPeer);
  Serial.println("EspNOW intialized");
  initBar30(bar30);
  Serial.println("Bar30 initialised");
  initVL6180X(irSensor);
  Serial.println("Laser sensor initialised");

  if (!rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }

  initLittleFS();
  file = LittleFS.open(FILE_NAME, FILE_WRITE, true);

  timer = timerBegin(0, 80, true);
  if (timer == NULL) {
    Serial.println("Failed to initialize timer");
    return;
  } else {
    Serial.println("Timer intialized");
  }
  timerAttachInterrupt(timer, &onTimer, false);
  timerAlarmWrite(timer, STEPPER_TIMER_TIME, true);
  timerAlarmEnable(timer);

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  irSensor.setTimeout(100);

  // go up
  while (readDistance(irSensor) < DISTANCE_UPPER_LIMIT) {
    moveMotorUp();
  }
  stopMotor();
  delay(10 * 1000);

  // send initial data to station
  float depth = updateDepth(bar30);
  DateTime now = rtc.now();

  String msg = "EX01 ";
  uint8_t hour = now.hour();
  if (hour < 10)
    msg.concat('0');
  msg.concat(hour);
  
  msg.concat(':');
  uint8_t min = now.minute();
  if (min < 10)
    msg.concat('0');
  msg.concat(min);
  
  msg.concat(':');
  uint8_t sec = now.second();
  if (sec < 10)
    msg.concat('0');
  msg.concat(sec);
  msg.concat(' ');

  msg.concat(depth);
  msg.concat(" m\n");

  espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());

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

  resetReadingsCount();
}

bool done = false;
bool firstTrial = true;
bool calibrating = false;
bool mission = false;
void loop() {
  ArduinoOTA.handle();

  if (!mission && !calibrating) {
    if (q.size()) {
      String command = q.front();
      q.pop();
      if (command == "calibration") {
        calibrating = true;
        String msg = "Starting calibration";
        espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
        Serial.println("Starting calibration");
      }
      else if (command == "mission") {
        mission = true;
        String msg = "Starting mission";
        espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
        Serial.println("Starting mission");
      }
    }

    return;
  }

  float depth = updateDepth(bar30);

  float requiredDistance;
  if (mission)
    requiredDistance = getRequiredDistance(depth);
  else if (calibrating)
    requiredDistance = NEUTRAL_POINT;

  Serial.printf("required distance: %f\n", requiredDistance);
  if (readDistance(irSensor) > requiredDistance + 2) {
    // go down
    moveMotorDown();
  } else if (readDistance(irSensor) < requiredDistance - 2) {
    // go up
    moveMotorUp();
  } else {
    // stop motor
    stopMotor();
  }

  if (calibrating)
    return;
  
  // if (firstTrial)
  //   depth = 1.0f;
  // else
  //   depth = 1.45f;

  String msg = writeMsg(file, rtc.now(), depth);
  if (msg != "") {
    espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
  }

  done = isDone(depth, msg != "");
  if (done) {
    Serial.println("DONE!");
    while (readDistance(irSensor) < DISTANCE_UPPER_LIMIT) {
      ArduinoOTA.handle();
      moveMotorUp();
      
      String msg = writeMsg(file, rtc.now(), depth);
      if (msg != "") {
        espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
      }
    }
    stopMotor();

    unsigned long time = millis();
    while (millis() - time <= 30 * 1000) {
      ArduinoOTA.handle();
      String msg = writeMsg(file, rtc.now(), depth);
      if (msg != "") {
        espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
      }
    }

    Serial.println("starting sending");

    file.close();
    file = LittleFS.open(FILE_NAME, FILE_READ, false);
    String msg = "PROFILE_START";
    espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
    while (file.available()) {
      msg = file.readStringUntil('\n');
      espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
      // Serial.println(depth);
    }
    msg = "PROFILE_END";
    espNowSend(espNowPeer, (uint8_t*)msg.c_str(), msg.length());
    file.close();
    file = LittleFS.open(FILE_NAME, FILE_WRITE, false);

    done = false;
    firstTrial = false;
    resetReadingsCount();
    resetFileWriteTime();
  }
}

#endif

// milestone 1 -> float & rov down
// milestone 2 -> main rov & camera settings
// milestone 3 -> photosphere, measurements, map
// milestone 4 -> tests, other settings, hotkeys, etc.
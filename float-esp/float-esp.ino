#include <Wire.h>
#include <MS5837.h>
#include <LittleFS.h>
#include <esp_now.h>
#include <WiFi.h>
// #include <EEPROM.h>
// #include <TimerOne.h>

#define dirPin 32
#define stepPin 33
#define potPin 34
#define potLowerLimit 20
#define potUpperLimit 1000
#define dirUP HIGH
#define dirDown LOW
#define ENABLE_PIN 16
#define stepperStepTime 500

#define FLUID_DENSITY 997
#define SET_POINT 0.2f
#define HYSTERESIS 0.03f

volatile float initialDepth = 0;
volatile float depth = 0;
int lastDepthAddr = 0;

File file;
  esp_now_peer_info_t peer;

MS5837 sensor;
unsigned long totalTime = 0;
unsigned long lastReadingTime;
unsigned long lastTime;
bool timerEnded = false;
bool inRange = false;
bool stepperState = LOW;

int readPot() {
  int potVal = analogRead(potPin);
  return potVal;
}

void formatLittleFS() {
  Serial.println("Formatting LittleFS...");
  if (LittleFS.format()) {
    Serial.println("LittleFS formatted successfully.");
  } else {
    Serial.println("Failed to format LittleFS.");
  }
}

void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    formatLittleFS();  // Format LittleFS if mount fails
    if (!LittleFS.begin()) {
      Serial.println("LittleFS Mount Failed after formatting");
      return;
    }
  }

  file = LittleFS.open("/data.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
}

void initEspNow() {
  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  WiFi.mode(WIFI_STA);
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
}

void espNowSend(String data) {
  int index = 0;
  while (index < data.length()) {
    int sentSize = min(data.length() - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
    ESP_ERROR_CHECK(esp_now_send(peer.peer_addr, (uint8_t*)(data.c_str() + index), sentSize));
    index += sentSize;
  }
}

void sendDepthData() {
  file.close();
  file = LittleFS.open("/data.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  while(file.available()) {
    espNowSend(file.readStringUntil('\n'));
    delay(20);
  }

  file.close();

  file = LittleFS.open("/data.txt", FILE_WRITE);
}

void readPressureSensor() {
  sensor.read();

  Serial.print("Pressure: ");
  Serial.print(sensor.pressure());
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(sensor.temperature());
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(sensor.altitude());
  Serial.println(" m above mean sea level");
}

void toggleStep(){
  stepperState = !stepperState;
  digitalWrite(stepPin, stepperState);
}

void updateDepth(){
  sensor.read();
  depth = sensor.depth();

  if(initialDepth <= 0) {
    depth = depth - initialDepth;
  } else {
    depth = depth + initialDepth;
  }

  // espNowSend(String(depth));
  file.printf("%f\n", depth);
  // Serial.println(depth);
}

unsigned long timer2;

void setup() {
  Serial.begin(115200);

  // Initializing pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(13, OUTPUT);


  initLittleFS();
  initEspNow();
  initBar30();
  
  sensor.read();
  initialDepth = sensor.depth();
  Serial.print("initial Depth = ");
  Serial.println(initialDepth);

  totalTime = millis();
  lastReadingTime = millis();
  
  // go up
  // digitalWrite(ENABLE_PIN, LOW);
  // digitalWrite(dirPin, dirUP);
  // while(readPot() <= potUpperLimit){
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(stepperStepTime);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(stepperStepTime);
  // }
  
  // Timer1.initialize(stepperStepTime);
  // Timer1.attachInterrupt(toggleStep);
  timer2 = millis();
}


void loop() {
  // ------------------- hysteresis control -------------------
    updateDepth();
    // storeEEPROM();

    if (millis() - timer2 >= 30000) {
      Serial.println("Sending");
      sendDepthData();
      Serial.println("Stopped Sending");
      timer2 = millis();
    }


    // if(!timerEnded){
    //   // go up
    //   if(totalTime >= 45000000){
    //     timerEnded = true;
    //   }
    //   else if ((depth > SET_POINT + HYSTERESIS) && (readPot() <= potUpperLimit)) {
    //     digitalWrite(ENABLE_PIN, LOW);
    //     digitalWrite(dirPin, dirUP);
    //     stepperState = HIGH;
    //   }
    //   // go down
    //   else if ((depth < SET_POINT - HYSTERESIS) && (readPot() >= potLowerLimit)) {
    //     digitalWrite(ENABLE_PIN, LOW);
    //     digitalWrite(dirPin, dirDown);
    //     stepperState = HIGH;
    //   }
    //   // float is in range
    //   else if (depth >= SET_POINT - HYSTERESIS && depth <= SET_POINT + HYSTERESIS) {
    //     digitalWrite(ENABLE_PIN, HIGH);
    //     totalTime += micros() - lastTime;
    //   }
    //   else{
    //     digitalWrite(ENABLE_PIN, HIGH);
    //   }
    //   lastTime = micros();
    // } else {
    //   // timer finished
    //   if (readPot() <= potUpperLimit) {
    //     digitalWrite(ENABLE_PIN, LOW);
    //     digitalWrite(dirPin, dirUP);
    //     stepperState = HIGH;
    //   } else {
    //     digitalWrite(ENABLE_PIN, HIGH);
    //   }
    // }
  
}

#if 0
#include <Wire.h>
#include <VL6180X.h>
#include "MS5837.h"
#include "RTClib.h"
#include <esp_timer.h>

#define DIR_PIN 32
#define STEP_PIN 33
#define DIR_UP HIGH
#define DIR_DOWN LOW
#define ENABLE_PIN 23

#define STEPPER_STEP_TIME 70
#define STEPPER_TIMER_TIME 100

hw_timer_t *timer = NULL;
VL6180X irSensor;
MS5837 sensor;
RTC_DS3231 rtc;

bool stopped = true;
bool stepperState = false;

void IRAM_ATTR onTimer() {
  if (!stopped) {
    stepperState = !stepperState;
    digitalWrite(STEP_PIN, stepperState);
  }
}

void initPins() {
  // pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);  
}

void moveMotorUp() {
  stopped = false;
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_UP);
  // Serial.println("moving motor up");
}

void moveMotorDown() {
  stopped = false;
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_DOWN);
  // Serial.println("moving motor down");
}

void stopMotor() {
  stopped = true;
  digitalWrite(ENABLE_PIN, HIGH);
  // Serial.println("stopping motor");
}

void initRTC() {
  if (! rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }
}

void initBar30() {
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  sensor.read();
  // initialDepth = abs(sensor.depth());
  // Serial.print("initial Depth = ");
  // Serial.println(initialDepth);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

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

  moveMotorUp();

  initBar30();
  initRTC();

  irSensor.init();
  irSensor.configureDefault();
  irSensor.setTimeout(100);

  Serial.println("VL6180X sensor ready.");
}

void loop() {
  int distance = irSensor.readRangeSingleMillimeters();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  Serial.print("depth: ");
  Serial.println(sensor.depth());

  DateTime now = rtc.now();
  Serial.print("day: ");
  Serial.println(now.day());
}
#endif
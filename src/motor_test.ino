#if 0
#include <esp_timer.h>
#include <string>

#define DIR_PIN 32
#define STEP_PIN 33
#define DIR_UP HIGH
#define DIR_DOWN LOW
#define ENABLE_PIN 23
#define STEPPER_STEP_TIME 70
#define STEPPER_TIMER_TIME 100

hw_timer_t *timer = NULL;
bool stopped = false;
bool stepperState = false;


void initPins() {
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

void IRAM_ATTR onTimer() {
  if (!stopped) {
    stepperState = !stepperState;
    digitalWrite(STEP_PIN, stepperState);
  }
}

void setup() {
  Serial.begin(115200);
  initPins();
  Serial.println("Pins intialized");

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

  // go up
  while (true) {
    moveMotorUp();
  }
}

void loop() {
}
#endif

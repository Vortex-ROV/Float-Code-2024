#include "stepper.h"
#include <Arduino.h>

bool static stopped = false;

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
	static bool stepperState = false;
    if (!stopped) {
      stepperState = !stepperState;
      digitalWrite(STEP_PIN, stepperState);
    }
}
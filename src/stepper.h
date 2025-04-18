#pragma once

#include <esp_attr.h>

#define DIR_PIN 32
#define STEP_PIN 33
#define DIR_UP LOW
#define DIR_DOWN HIGH
#define ENABLE_PIN 23
#define STEPPER_STEP_TIME 70
#define STEPPER_TIMER_TIME 100

void moveMotorUp();
void moveMotorDown();
void stopMotor();
void IRAM_ATTR onTimer();

#include <Wire.h>
#include <MS5837.h>
#include <esp_timer.h>


#define dirPin 32
#define stepPin 33
#define potPin 34
#define potLowerLimit 143
#define potUpperLimit 2900
#define dirUP HIGH
#define dirDown LOW
#define ENABLE_PIN 25
#define stepperStepTime 80

#define FLUID_DENSITY 997
#define SET_POINT 1.5f
#define HYSTERESIS 0.2f

hw_timer_t *timer = NULL;
volatile bool stepperState = HIGH;

void IRAM_ATTR onTimer() {
  stepperState = !stepperState;
  digitalWrite(stepPin, stepperState);
}
volatile float initialDepth = 0;
volatile float depth = 0;
int lastDepthAddr = 0;

MS5837 sensor;
unsigned long long trialTime = 0;
unsigned long long lastReadingTime;
unsigned long long lastTime;
bool timerEnded = false;
bool inRange = false;

int readPot(){
  return analogReadMilliVolts(potPin);
}

void toggleStep(){
  stepperState = !stepperState;
  digitalWrite(stepPin, stepperState);
}

void updateDepth(){
    sensor.read();
    depth = sensor.depth();
    if(initialDepth <= 0){
      depth = depth - initialDepth;
    } else {
      depth = depth + initialDepth;
    }
}


void setup() {
  // Initializing pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  Serial.begin(115200);


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
  initialDepth = sensor.depth();
  Serial.print("initial Depth = ");
  Serial.println(initialDepth);

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
  }
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer,100,true, 0);

  lastTime = micros(); // to account for setup time.
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
        stepperState = HIGH;
      }
      // go down
      else if ((depth < SET_POINT - HYSTERESIS) && (readPot() >= potLowerLimit)) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(dirPin, dirDown);
        stepperState = HIGH;
      }
      // float is in range
      else if (depth >= SET_POINT - HYSTERESIS && depth <= SET_POINT + HYSTERESIS) {
        digitalWrite(ENABLE_PIN, HIGH);
        trialTime += micros() - lastTime;
      }
      else{
        digitalWrite(ENABLE_PIN, HIGH);
      }
      lastTime = micros();
    } else {
      // timer finished
      if (readPot() <= potUpperLimit) {
        digitalWrite(ENABLE_PIN, LOW);
        digitalWrite(dirPin, dirUP);
        stepperState = HIGH;
      } else {
        digitalWrite(ENABLE_PIN, HIGH);
      }
    }
  
}

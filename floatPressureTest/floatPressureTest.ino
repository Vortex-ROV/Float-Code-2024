#include <Wire.h>
#include <MS5837.h>
#include <EEPROM.h>
#include <TimerOne.h>

#define dirPin 10
#define stepPin 11
#define potPin A1
#define potLowerLimit 20
#define potUpperLimit 1000
#define dirUP LOW
#define dirDown HIGH
#define ENABLE_PIN 8
#define stepperStepTime 500

#define FLUID_DENSITY 997
#define SET_POINT 0.2f
#define HYSTERESIS 0.03f
volatile float initialDepth = 0;
volatile float depth = 0;
int lastDepthAddr = 0;

MS5837 sensor;
unsigned long time = 0;
unsigned long lastReadingTime;
unsigned long lastTime;
bool timerEnded = false;
bool inRange = false;
bool stepperState = LOW;

int readPot(){
  int potVal = analogRead(potPin);
  return potVal;
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
    Serial.println(depth);
    if(initialDepth <= 0){
      depth = depth - initialDepth;
    } else {
      depth = depth + initialDepth;
    }
}

void storeEEPROM(){
    if((millis()-lastReadingTime)>2000){
    EEPROM.update(lastDepthAddr, (byte)(-depth*100));
    Serial.print("Stored A value");
    Serial.print(depth);
    Serial.println("in EEPROM");
    lastDepthAddr = lastDepthAddr>=255 ? 0:(lastDepthAddr+1);
    lastReadingTime = millis();
  }
}

void setup() {
  // Initializing pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(9600);

  Serial.println("Last Depth Readings:");
  for(int i = 0; i<=255; i++){
    Serial.println(EEPROM.read(i));
  }

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

  time = millis();
  lastReadingTime = millis();
  
  // go up
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(dirPin, dirUP);
  while(readPot() <= potUpperLimit){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepperStepTime);
      }
  
  Timer1.initialize(stepperStepTime);
  Timer1.attachInterrupt(toggleStep);
}

void loop() {
  // ------------------- hysteresis control -------------------
    updateDepth();
    storeEEPROM();

    if(!timerEnded){
      // go up
      if(time >= 45000000){
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
        time += micros() - lastTime;
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

#include <Wire.h>
#include <MS5837.h>
#include <EEPROM.h>

#define dirPin 10
#define stepPin 11
#define potPin A1
#define enablePin 8
#define potLowerLimit 20
#define potUpperLimit 1000
#define dirUP HIGH
#define dirDown LOW
#define ENABLE_PIN 8
#define stepperStepTime 500

#define FLUID_DENSITY 997
#define SET_POINT 1.5f
#define HYSTERESIS 0.2f
volatile float initialDepth = 0;
int lastDepthAddr = 0;

MS5837 sensor;
unsigned long time;
unsigned long lastReadingTime;
bool timerStarted = false;

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

void goUp() {
  digitalWrite(dirPin, dirUP);
  if (readPot() <= potUpperLimit) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepperStepTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepperStepTime);
  }
}

void goDown() {
  digitalWrite(dirPin, dirDown);
  if (readPot() >= potLowerLimit) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepperStepTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepperStepTime);
  }
}

void stop() {
  digitalWrite(stepPin, LOW);
  Serial.println("Stopped!");
}

void setup() {
  // put your setup code here, to run once:
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

  digitalWrite(ENABLE_PIN, LOW);
  
  // go up
  goUp();
}


void loop() {
  // ------------------- hysteresis control -------------------
  while (true) {
    sensor.read();
    float depth = sensor.depth();
    Serial.print("rawdepth = ");
    Serial.println(depth);
    if(initialDepth <= 0){
      depth = depth - initialDepth;
    } else {
      depth = depth + initialDepth;
    }
    bool stopped = true;

    // storing depth values in EEPROM every two seconds
    if((millis()-lastReadingTime)>2000){
      EEPROM.update(lastDepthAddr, (byte)(-depth*100));
      Serial.print("Stored A value");
      Serial.print(depth);
      Serial.println("in EEPROM");
      lastDepthAddr = lastDepthAddr>=255 ? 0:(lastDepthAddr+1);
      lastReadingTime = millis();
    }

    // go up
    if ((depth > SET_POINT + HYSTERESIS) && (readPot() <= potUpperLimit)) {
      digitalWrite(ENABLE_PIN, LOW);
      digitalWrite(dirPin, dirUP);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepperStepTime);
      stopped = false;
    }

    // go down
    if ((depth < SET_POINT - HYSTERESIS) && (readPot() >= potLowerLimit)) {
      digitalWrite(ENABLE_PIN, LOW);
      digitalWrite(dirPin, dirDown);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, LOW);
      stopped = false;
    }

  // float is in range
  if (depth >= SET_POINT - HYSTERESIS && depth <= SET_POINT + HYSTERESIS && !timerStarted) {
    time = millis();
    timerStarted = true;
  }

  // timer finished
  if (timerStarted && millis() - time > 45000) {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(dirPin, dirUP);

    // keep going up until we can't or we reach the top of the pool
    while(depth >= 0.2 && readPot() <= potUpperLimit) {
      depth readings here are not relative to starting point
      sensor.read();
      depth = sensor.depth();
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepperStepTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepperStepTime);
    }

    // stop the motor
    digitalWrite(ENABLE_PIN, HIGH);

    // make sure we are at the top of the pool
    while (depth >= 0.2) {
      depth = sensor.depth();
    }

    timerStarted = false;
  }

  if (stopped) {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}

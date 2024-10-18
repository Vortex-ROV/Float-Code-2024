#include <Wire.h>
#include <MS5837.h>

#define dirPin 10
#define stepPin 11
#define potPin A1
#define enablePin 8
#define potLowerLimit 20
#define potUpperLimit 1000
#define dirUP HIGH
#define dirDown LOW
#define ENABLE_PIN 8

#define FLUID_DENSITY 997
#define SET_POINT 1.5f
#define HYSTERESIS 0.2f

MS5837 sensor;
unsigned long time;
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
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}

void goDown() {
  digitalWrite(dirPin, dirDown);
  if (readPot() >= potLowerLimit) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
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

  time = millis();

  digitalWrite(ENABLE_PIN, LOW);
  
  // go up
  // digitalWrite(dirPin, dirUP);
  // while(readPot()<= potUpperLimit) {
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(1000);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(1000);
  // }
}


void loop() {
  // put your main code here, to run repeatedly:
  // goUp();
  // digitalWrite(dirPin, dirUP);
  // while(readPot()<= potUpperLimit){
  //       digitalWrite(stepPin, HIGH);
  //       delayMicroseconds(1000);
  //       digitalWrite(stepPin, LOW);
  //       delayMicroseconds(1000);
  // }
  // delay(2000);
  // // goDown();
  // digitalWrite(dirPin, dirDown);
  // while(readPot() >= potLowerLimit){
  //   digitalWrite(stepPin, HIGH);
  //   delayMicroseconds(1000);
  //   digitalWrite(stepPin, LOW);
  //   delayMicroseconds(1000);
  // }
  // delay(2000);

  // ------------------- hysteresis control -------------------
  float depth = sensor.depth();
  bool stopped = true;

  // go up
  if ((depth > SET_POINT + HYSTERESIS) && (readPot() <= potUpperLimit)) {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(dirPin, dirUP);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    stopped = false;
  }

  // go down
  if ((depth < SET_POINT - HYSTERESIS) && (readPot() >= potLowerLimit)) {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(dirPin, dirDown);
    delayMicroseconds(1000);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
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
      depth = sensor.depth();
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
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

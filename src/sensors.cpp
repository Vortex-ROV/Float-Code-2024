#include "sensors.h"
#include <Wire.h>

#define FLUID_DENSITY 997
static float lastDepth = 0.0f;
static float initialDepth = 0.0f;

void initBar30(MS5837& sensor) {
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

    const int readingsCount = 10;
    for (int i = 0; i < readingsCount; i++) {
      initialDepth += sensor.depth();
      delay(100);
    }

    initialDepth /= readingsCount;
}

void initVL6180X(VL6180X& sensor) {
    Wire.begin();
  
    sensor.init();  // Just call the function without checking a return value
    sensor.configureDefault();
    Serial.println("VL6180X Initialized.");
}

int readDistance(VL6180X& irSensor) {
    static float laser = irSensor.readRangeSingleMillimeters();
    laser = irSensor.readRangeSingleMillimeters() * 0.01f + laser * 0.99f;
    // Serial.print("Distance: ");
    // Serial.print(laser);
    // Serial.println(" mm");
    return laser;
}

float updateDepth(MS5837& bar30) {
    static int lastReadingTime = 0;
    if (millis() - lastReadingTime < 15)
      return lastDepth;
      
    bar30.read();
    // Serial.printf("bar30: %f\n", bar30.depth());
    // float depth = bar30.depth() - initialDepth + 0.335;
    float depth = bar30.depth() - initialDepth;
    
    lastReadingTime = millis();
    lastDepth = depth;
    return depth;
}

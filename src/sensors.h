#pragma once

#include "MS5837.h"
#include "VL6180X.h"

#define DISTANCE_LOWER_LIMIT 65
#define DISTANCE_UPPER_LIMIT 90

void initBar30(MS5837& sensor);
void initVL6180X(VL6180X& sensor);
int readDistance(VL6180X& irSensor);
float updateDepth(MS5837& bar30);

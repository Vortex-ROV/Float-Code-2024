#pragma once

#define MAX_DEPTH 2.0f
#define NEUTRAL_POINT ((DISTANCE_LOWER_LIMIT + DISTANCE_UPPER_LIMIT) / 2.0f)
#define REQUIRED_DEPTH 1.5f
#define MARGIN 0.5f
// #define PROFILE_TIME 45 * 1000
#define PROFILE_READINGS 10

float getRequiredDistance(float depth);
bool isDone(float depth, bool written);
void resetReadingsCount();
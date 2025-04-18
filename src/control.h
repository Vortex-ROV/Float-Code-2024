#pragma once

#define MAX_DEPTH 2.5f
#define NEUTRAL_POINT 75
#define REQUIRED_DEPTH 1.0f
#define MARGIN 0.5f
#define PROFILE_TIME 45 * 1000

float getRequiredDistance(float depth);
bool isDone(float depth);
void resetAccumulatedTime();
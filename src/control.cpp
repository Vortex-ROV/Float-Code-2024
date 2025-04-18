#include "control.h"
#include "sensors.h"

static unsigned long lastTime = 0;
static unsigned long accumulatedTime = 0;

float getRequiredDistance(float depth) {
    float requiredDistance;
    float minDepth = 0.0f;

    if (depth > REQUIRED_DEPTH)
        return (depth - REQUIRED_DEPTH) / (MAX_DEPTH - REQUIRED_DEPTH) * (DISTANCE_UPPER_LIMIT - NEUTRAL_POINT) + NEUTRAL_POINT;

    return depth / (REQUIRED_DEPTH - minDepth) * (NEUTRAL_POINT - DISTANCE_LOWER_LIMIT) + DISTANCE_LOWER_LIMIT;
}

bool isDone(float depth) {
    if (depth >= REQUIRED_DEPTH - MARGIN && depth <= REQUIRED_DEPTH + MARGIN) {
        unsigned long time = millis();
        accumulatedTime += (time - lastTime);
        lastTime = time;

        if (accumulatedTime > PROFILE_TIME) {
            return true;
        }
    }
    return false;
}

void resetAccumulatedTime() {
    lastTime = millis();
    accumulatedTime = 0;
}

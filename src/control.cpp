#include "control.h"
#include "sensors.h"

// static unsigned long lastTime = 0;
// static unsigned long accumulatedTime = 0;
static unsigned int totalReadingsInRange = 0;
static bool lastReadingWrittenInRange = false;

float getRequiredDistance(float depth) {
    float requiredDistance;
    float minDepth = 0.0f;

    if (depth > REQUIRED_DEPTH) {
        requiredDistance = (depth - REQUIRED_DEPTH) / (MAX_DEPTH - REQUIRED_DEPTH) * (DISTANCE_UPPER_LIMIT - NEUTRAL_POINT) + NEUTRAL_POINT;
        requiredDistance = min(requiredDistance, (float)DISTANCE_UPPER_LIMIT);
        requiredDistance = max(requiredDistance, (float)DISTANCE_LOWER_LIMIT);
        return requiredDistance;
    }
    requiredDistance = depth / (REQUIRED_DEPTH - minDepth) * (NEUTRAL_POINT - DISTANCE_LOWER_LIMIT) + DISTANCE_LOWER_LIMIT;
    requiredDistance = min(requiredDistance, (float)DISTANCE_UPPER_LIMIT);
    requiredDistance = max(requiredDistance, (float)DISTANCE_LOWER_LIMIT);
    return requiredDistance;
}

bool isDone(float depth, bool written) {
    bool readingInRange = (depth + 0.7) >= 2.0f && (depth + 0.7) <= 3.0f;
    // bool readingInRange = depth >= REQUIRED_DEPTH - MARGIN && depth <= REQUIRED_DEPTH + MARGIN;
    if (lastReadingWrittenInRange && readingInRange && written) {
        totalReadingsInRange++;
    }

    if (written)
        lastReadingWrittenInRange = readingInRange;

    return totalReadingsInRange >= PROFILE_READINGS;
}

// lastTime = millis();
void resetReadingsCount() {
    // accumulatedTime = 0;

    lastReadingWrittenInRange = false;
    totalReadingsInRange = 0;
}

#include "control.h"
#include "sensors.h"

// static unsigned long lastTime = 0;
// static unsigned long accumulatedTime = 0;
static unsigned int totalReadingsInRange = 0;
static bool lastReadingWrittenInRange = false;

float getRequiredDistance(float depth) {
    float requiredDistance;
    float minDepth = 0.0f;

    if (depth > REQUIRED_DEPTH)
        return (depth - REQUIRED_DEPTH) / (MAX_DEPTH - REQUIRED_DEPTH) * (DISTANCE_UPPER_LIMIT - NEUTRAL_POINT) + NEUTRAL_POINT;

    return depth / (REQUIRED_DEPTH - minDepth) * (NEUTRAL_POINT - DISTANCE_LOWER_LIMIT) + DISTANCE_LOWER_LIMIT;
}

bool isDone(float depth, bool written) {
    bool readingInRange = depth >= REQUIRED_DEPTH - MARGIN && depth <= REQUIRED_DEPTH + MARGIN;
    if (lastReadingWrittenInRange && readingInRange && written) {
        totalReadingsInRange++;
    }

    lastReadingWrittenInRange = readingInRange && written;
    return totalReadingsInRange >= PROFILE_READINGS;
}

// lastTime = millis();
void resetReadingsCount() {
    // accumulatedTime = 0;

    lastReadingWrittenInRange = false;
    totalReadingsInRange = 0;
}

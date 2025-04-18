#include "filesystem.h"

static unsigned long lastTime = 0;

void initLittleFS() {
    LittleFS.begin();
    Serial.println("Formatting LittleFS...");
    if (LittleFS.format()) {
        Serial.println("LittleFS formatted successfully.");
    } else {
        Serial.println("Failed to format LittleFS.");
    }
}

void resetFileWriteTime() {
    lastTime = 0;
}

void writeMsg(File &file, DateTime now, float depth) {
    if (millis() - lastTime < 5 * 1000)
        return;

    lastTime = millis();

    String msg = "EX01 ";

    uint8_t hour = now.hour();
    if (hour < 10)
        msg.concat('0');
    msg.concat(hour);
    
    msg.concat(':');
    uint8_t min = now.minute();
    if (min < 10)
        msg.concat('0');
    msg.concat(min);
    
    msg.concat(':');
    uint8_t sec = now.second();
    if (sec < 10)
        msg.concat('0');
    msg.concat(sec);
    msg.concat(' ');

    msg.concat(depth);
    msg.concat(" m");

    msg.concat('\n');
    file.write((const uint8_t*)msg.c_str(), msg.length());
}

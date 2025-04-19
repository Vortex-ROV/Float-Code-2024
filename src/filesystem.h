#pragma once

#define FILE_NAME "/data"
#include <LittleFS.h>
#include <RTClib.h>

void initLittleFS();
void resetFileWriteTime();
String writeMsg(File &file, DateTime now, float depth);

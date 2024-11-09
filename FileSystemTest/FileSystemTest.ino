#include <LittleFS.h>
File file;

void setup() {
  Serial.begin(115200); // Start serial communication
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }
  LittleFS.format();
  // // Open file for reading
  // file = LittleFS.open("/data.txt", FILE_READ);
  // if (!file) {
  //   Serial.println("Failed to open file for reading");
  // } else {
  //   while (file.available()) {
  //     Serial.println(file.readStringUntil('\n'));
  //     delay(20);
  //   }
  //   file.close();
  // }

  // // Open file for writing
  // file = LittleFS.open("/data.txt", FILE_APPEND);
  // if (!file) {
  //   Serial.println("Failed to open file for writing");
  // } else {
  //   file.printf("AHHHHHH");
  //   file.flush(); // Ensure data is written
  //   file.close();
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
}

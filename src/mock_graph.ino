#if 0

void setup() {
    Serial.begin(115200);
}

void loop() {
  static float t = 0.0;
  static unsigned long startTime = millis();  // Store the start time

  // Calculate sine wave (shifted to positive range)
  float sineValue = (sin(t) + 1.0) * 50;  // Sine wave from 0 to 100

  // Calculate time in HH:MM:SS format
  unsigned long elapsedTime = (millis() - startTime) / 1000;  // Convert ms to sec
  int hours = (elapsedTime / 3600) % 24;
  int minutes = (elapsedTime / 60) % 60;
  int seconds = elapsedTime % 60;

  // Print everything using ONE Serial.println
  String msg = "EX06 ";
  msg.concat(hours < 10 ? "0" : "");
  msg.concat(hours);
  msg.concat(":");
  msg.concat(minutes < 10 ? "0" : "");
  msg.concat(minutes);
  msg.concat(":");
  msg.concat(seconds < 10 ? "0" : "");
  msg.concat(seconds);
  msg.concat(" ");
  msg.concat(sineValue);
  msg.concat(" m");

  Serial.println(msg);

  t += 0.1;   // Increment sine wave phase
  delay(100); // Wait 1 second
}

#endif
#define arduino

#ifdef esp32
#define dirPin 26
#define stepPin 25
#endif

#ifdef arduino
#define dirPin 10
#define stepPin 11
#endif

#define stepsPerRevolution 1000

int noOfSteps = 0;

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  }

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'r') {
      // Rotate in one direction continuously
      digitalWrite(dirPin, LOW);
      while (true) {
        // Step forward
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
        noOfSteps++;


        // Check if there's new command
        if (Serial.available()) {
          char newCommand = Serial.read();
          if (newCommand == 'l' || newCommand == '0') {
            Serial.print("Number of Steps = ");
            Serial.println(noOfSteps);
            noOfSteps = 0;
            break;  // Exit the loop to change direction or stop
          }
        }
      }
    }

    else if (command == 'l') {
      // Rotate in the opposite direction continuously
      digitalWrite(dirPin, HIGH);
      while (true) {
        // Step backward
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
        noOfSteps++;
        // Check if there's new command
        if (Serial.available()) {
          char newCommand = Serial.read();
          if (newCommand == 'r' || newCommand == '0') {
            Serial.print("Number of Steps = ");
            Serial.println(noOfSteps);
            noOfSteps = 0;
            break;  // Exit the loop to change direction or stop
          }
        }
      }
    }
  }
}

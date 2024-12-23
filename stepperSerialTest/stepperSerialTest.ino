#define esp32

#ifdef esp32
#define dirPin 32
#define stepPin 33
#define enablePin 23
#define adcPin  34
#endif

#ifdef arduino
#define dirPin 10
#define stepPin 11
#endif


void printPot(){
  int adcValue = analogRead(adcPin);
  int adcMillivolts = analogReadMilliVolts(adcPin);
  
  // Print the ADC value to the Serial Monitor
  Serial.print("ADC Value: ");
  Serial.println(adcValue);
  Serial.print("ADC Value Millivolts: ");
  Serial.println(adcMillivolts);
}
int noOfSteps = 0;

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(adcPin, INPUT);
  digitalWrite(enablePin, LOW);
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
        delayMicroseconds(100);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(100);
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
        // Step  backward
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(100);
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
    else if (command == 'p'){
      printPot();
    }
  } else {
    printPot();
  }
}

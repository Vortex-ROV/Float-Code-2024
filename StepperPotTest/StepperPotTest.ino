#define dirPin 32
#define stepPin 33
#define potPin 34
#define enablePin 25
#define potLowerLimit 143
#define potUpperLimit 3164
#define dirUP HIGH
#define dirDown LOW
#define stepperStepTime 80


void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
}

int readPot(){
  int potVal = analogReadMilliVolts(potPin);
  return potVal;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dirPin, dirUP);
  while(readPot()<= potUpperLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepperStepTime);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepperStepTime);
  }
  delay(1000);
  digitalWrite(dirPin, dirDown);
  while(readPot() >= potLowerLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepperStepTime);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepperStepTime);
  }
  delay(1000);
}

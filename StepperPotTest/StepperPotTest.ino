#define dirPin 10
#define stepPin 11
#define potPin A1
#define potLowerLimit 15
#define potUpperLimit 1000
#define dirUP HIGH
#define dirDown LOW
#define stepperStepTime 400


void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
}

int readPot(){
  int potVal = analogRead(potPin);
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

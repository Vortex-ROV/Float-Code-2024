#define dirPin 10
#define stepPin 11
#define potPin A1
#define enablePin 8
#define potLowerLimit 15
#define potUpperLimit 1000
#define dirUP LOW
#define dirDown HIGH

void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
}

int readPot(){
  int potVal = analogRead(potPin);
  return potVal;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dirPin, dirDown);
  while(readPot()<= potUpperLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  }
  delay(1000);
  digitalWrite(dirPin, dirUP);
  while(readPot() >= potLowerLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  }
  delay(1000);
}

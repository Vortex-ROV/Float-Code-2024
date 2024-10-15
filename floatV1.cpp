#define dirPin 10
#define stepPin 11
#define potPin A0
#define potLowerLimit 90
#define potUpperLimit 970
#define dirUP LOW
#define dirDown HIGH

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
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  }
  delay(90000);
  digitalWrite(dirPin, dirDown);
  while(readPot() >= potLowerLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
  }
  delay(90000);
}

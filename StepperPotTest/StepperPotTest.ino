#define dirPin 32
#define stepPin 33
#define potPin 34
#define enablePin 23
#define potLowerLimit 143
#define potUpperLimit 2900
#define dirUP HIGH
#define dirDown LOW
#define stepperStepTime 80


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
}

int readPot(){
  return analogReadMilliVolts(potPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dirPin, dirUP);
  int pot = readPot();
  while(pot <= potUpperLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepperStepTime);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepperStepTime);
        pot = readPot();
  }
  Serial.println(pot);
  delay(1000);
  digitalWrite(dirPin, dirDown);
  pot = readPot();
  while(pot >= potLowerLimit){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepperStepTime);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepperStepTime);
        pot = readPot();
  }
  Serial.println(pot);
  delay(1000);
}

#define esp32

#define DIR_PIN 16
#define PWM_PIN 17
#define adcPin 34

#define MOTOR_SPEED 255
#define DIR_UP LOW
#define DIR_DOWN HIGH

// 3200 - 120

float x = 0;
float y = 0;

void updatePot() {
  x = ( 99.0 * x + (analogRead(34)/4)) / 100.0;
  y = ( x * 4095.0 /1134.0);
}

void printPot() {
  Serial.print("ADC Value Millivolts: ");
  Serial.println(y);
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(adcPin, INPUT);
}

void moveMotorUp() {
  digitalWrite(DIR_PIN, DIR_UP);
  analogWrite(PWM_PIN, MOTOR_SPEED);
}

void moveMotorDown() {
  digitalWrite(DIR_PIN, DIR_DOWN);
  analogWrite(PWM_PIN, MOTOR_SPEED);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
}

void loop() {
  updatePot();
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'u') {
      moveMotorUp();
      Serial.println("up");
    } else if (command == 'd') {
      moveMotorDown();
      Serial.println("down");
    } else if (command == 'p'){
      stopMotor();
      printPot();
    }
  }
}

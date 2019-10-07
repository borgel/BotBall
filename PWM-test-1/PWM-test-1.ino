// https://www.pololu.com/product/2135

const int phaseAPin = 15;
const int enableAPin = 16;
const int phaseBPin = 18;
const int enableBPin = 17;

void setup() {
  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  
  // 8 bits
}

void loop() {
  // rotate in one way
  // dir
  digitalWrite(phaseAPin, HIGH);
  digitalWrite(phaseBPin, LOW);

  //PWM
  analogWrite(enableAPin, 255);
  analogWrite(enableBPin, 64);
  delay(1000);

  // rotate the other way
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);
  
  analogWrite(enableAPin, 64);
  analogWrite(enableBPin, 255);

  delay(1000);
}

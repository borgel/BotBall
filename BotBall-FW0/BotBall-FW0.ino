// https://www.pololu.com/product/2135

const int pwmAPin = 18;
const int pwmBPin = 17;
const int pwmMirror = 16;
const int phaseAPin = 15;
const int phaseBPin = 18;

// single status LED
const int ledPin = 19;

void setup() {
  // status LED
  pinMode(ledPin, OUTPUT);
  
  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);

  // active low
  pinMode(pwmMirror, OUTPUT);
  //analogWrite(pwmAPin, 100);
}

void loop() {
  digitalWrite(ledPin, LOW);
  
  // rotate in one way
  // dir
  digitalWrite(phaseAPin, HIGH);
  digitalWrite(phaseBPin, LOW);

  //PWM
  // 8 bits
  analogWrite(pwmAPin, 255);
  analogWrite(pwmBPin, 64);
  analogWrite(pwmAPin, 1);
  delay(1000);
  digitalWrite(ledPin, HIGH);

  // rotate the other way
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);
  
  analogWrite(pwmAPin, 64);
  analogWrite(pwmBPin, 255);
  analogWrite(pwmAPin, 250);

  delay(1000);
}

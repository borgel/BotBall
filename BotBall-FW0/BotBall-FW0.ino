#include "IntervalTimer.h"
#include "rotary.h"

// motor driver
// https://www.pololu.com/product/2135

const int pwmAPin = 18;
const int pwmBPin = 17;
const int phaseAPin = 15;
const int phaseBPin = 18;

// single status LED
const int ledPin = 19;

// active low
const int ledRPin = 6;
const int ledGPin = 3;
const int ledBPin = 4;

void setup() {
  // status LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(ledRPin, INPUT);
  pinMode(ledGPin, INPUT);
  pinMode(ledBPin, INPUT);

  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);

  delay(2000);

  Serial.begin(115200);
  Serial.println("Booted...");

  rotary_Begin();
  
  //digitalWrite(ledPin, LOW);
  rotary_Home();
  //digitalWrite(ledPin, HIGH);
}

void loop() {
  /*
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);

  analogWrite(pwmAPin, 64);
  analogWrite(pwmBPin, 255);
  */
  
  //digitalWrite(ledPin, LOW);

  // FIXME rm
  rotary_Home();

  //digitalWrite(ledPin, HIGH);

  // rotate the other way
}

#include "IntervalTimer.h"
#include "rotary.h"
#include "hw.h"

// motor driver
// https://www.pololu.com/product/2135

void setup() {
  // status LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(ledRPin, OUTPUT);
  pinMode(ledGPin, OUTPUT);
  pinMode(ledBPin, OUTPUT);
  digitalWrite(ledRPin, HIGH);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledBPin, HIGH);

  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);

  delay(2000);

  Serial.begin(115200);
  Serial.println("Booted...");

  rotary_Begin();
  // do it again to avoid any partial homes
  rotary_Begin();
}

void loop() {
  rotary_Home();
  rotary_ScanContinuous();
  // only returns if it needs to re-home
  
  /*
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);

  analogWrite(pwmAPin, 64);
  analogWrite(pwmBPin, 255);
  */
}

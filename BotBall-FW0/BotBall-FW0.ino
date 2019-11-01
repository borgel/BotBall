/*
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
*/
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
#include <Wire.h>
extern TwoWire Wire1;
*/

#include "OcPwm.h"
#include "IntervalTimer.h"

// motor driver
// https://www.pololu.com/product/2135

const int pwmAPin = 18;
const int pwmBPin = 17;
const int pwmMirrorPin = 16;
const int phaseAPin = 15;
const int phaseBPin = 18;

// single status LED
const int ledPin = 19;

// active low
const int ledRPin = 6;
const int ledGPin = 3;
const int ledBPin = 4;

const int distanceNShutdown = 2;
const int distanceInt = 1;
const int distanceSDA = 23;
const int distanceSCL = 22;
SFEVL53L1X distanceSensor(Wire1, distanceNShutdown, distanceInt);

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

  //ocp_Setup(pwmMirrorPin);

  /*
  pinMode(distanceNShutdown, OUTPUT);
  digitalWrite(ledPin, LOW);
  delay(500);
  */

  pinMode(distanceInt, INPUT);
  pinMode(distanceSDA, OUTPUT);
  pinMode(distanceSCL, OUTPUT);

  /*
  digitalWrite(distanceSDA, LOW);
  digitalWrite(distanceSCL, LOW);
  delay(100);
  digitalWrite(distanceSDA, HIGH);
  digitalWrite(distanceSCL, HIGH);
  */
  
  Wire1.begin();
  Wire1.setSDA(distanceSDA);
  Wire1.setSCL(distanceSCL);

/*
  Wire1.beginTransmission(80);
  Wire1.write(0);  // address high byte
  Wire1.write(0);  // address low byte
  Wire1.endTransmission();
  */
  
    Serial.println("done sending");
    
  if(distanceSensor.begin() == 0) {
    Serial.println("sensor OK");
  }
  else {
    Serial.println("Failed to init distance sensor");
  }
}

void loop() {
  Serial.print(".");
  
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();
  Serial.print("Distance(mm): ");
  Serial.println(distance);
  
  digitalWrite(ledPin, LOW);

  ocp_SetDuty(30);

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

  ocp_SetDuty(200);

  // rotate the other way
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);

  analogWrite(pwmAPin, 64);
  analogWrite(pwmBPin, 255);
  analogWrite(pwmAPin, 250);

  delay(1000);
}

void homeSensor(void) {
}
}

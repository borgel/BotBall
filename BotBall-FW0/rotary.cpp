#include "rotary.h"
#include "OcPwm.h"

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include <Wire.h>

const int pwmMirrorPin = 16;

// TODO abstract these
const int distanceNShutdown = 2;
const int distanceInt = 1;
const int distanceSDA = 23;
const int distanceSCL = 22;

// 20 is about 4 revolutions/second
const int defaultMirrorSpeed = 20;

// below this (or low confidence) we consider the backstop
const int closeThreshMm = 35;
const int closeThreshHysteresisMm = 10;

// TODO IIR?
uint32_t homedDurationMs = 0;

SFEVL53L1X distanceSensor(Wire1, distanceNShutdown, distanceInt);

void rotary_Begin(void) {
  homedDurationMs = 0;
  
  Wire1.begin();
  Wire1.setSDA(distanceSDA);
  Wire1.setSCL(distanceSCL);
  
  if(distanceSensor.begin() != 0) {
    Serial.println("Failed to init distance sensor");
  }

  // let's not pretend
  distanceSensor.setDistanceModeShort();
  // TODO config window, etc
  // TODO what is reasonable?
  // options in ms are 15, 20, 33, 50, 100 (def), 200, 500
  distanceSensor.setTimingBudgetInMs(15);
  // >= timing budget
  distanceSensor.setIntermeasurementPeriod(15);
  distanceSensor.startRanging();

  // setup mirror motor
  ocp_Setup(pwmMirrorPin);
}

int inline getRange(void) {
  int const distance = distanceSensor.getDistance();
  // check getRangeStatus, return error if failed (0 success)
  if(distanceSensor.getRangeStatus() != 0) {
    return -1;
  }
  return distance;
}

void rotary_Home(void) {
  homedDurationMs = 0;
  
  int distance = 0;

  // start rotating at the normal speed
  // take take measurements
  // when measurement drops below close thresh, mark time
  // mark time when measurement exceeds close thresh
  // repeat X times to and print stuff

  Serial.println("Starting to home...");
  digitalWrite(19, LOW);
  
  ocp_SetDuty(defaultMirrorSpeed);
  
  Serial.printf("1=%dmm ", getRange());

  // spin until we start seeing the backstop
  do {
    distance = getRange();
  } while(distance > closeThreshMm + closeThreshHysteresisMm || distance == -1);
  int const backstopStart = millis();

  Serial.printf("2=%dmm ", distance);

  // make sure we wait until there has been at least one more measurement
  distanceSensor.checkForDataReady();

  // spin until we stop seeing the backstop
  do {
    distance = getRange();
  } while(distance <= closeThreshMm + closeThreshHysteresisMm || distance == -1);
  int const backstopStop = millis();

  Serial.printf("3=%dmm ", distance);
  
  digitalWrite(19, HIGH);
  homedDurationMs = backstopStop - backstopStart;
  Serial.printf("Backstop was %d ms wide\n", backstopStop - backstopStart);
}

void rotary_ScanContinuous(void) {
}

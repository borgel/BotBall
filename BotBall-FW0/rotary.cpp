#include "rotary.h"
#include "OcPwm.h"
#include "hw.h"

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
// slower than ~10 won't turn
const int defaultMirrorSpeed = 30;
// options in ms are 15, 20, 33, 50, 100 (def), 200, 500
const int timingBudgetMs = 20;

// below this (or low confidence) we consider the backstop
const int closeThreshMm = 40;
const int closeThreshHysteresisMm = 12;

// anything below this is probably bogus (too far, etc)
const int minimumSPAD = 800;

// TODO IIR?
uint32_t homedDurationMs = 0;

SFEVL53L1X distanceSensor(Wire1, distanceNShutdown, distanceInt);

void rotary_Begin(void) {
  homedDurationMs = 0;

  Wire1.begin();
  Wire1.setSDA(distanceSDA);
  Wire1.setSCL(distanceSCL);

  if (distanceSensor.begin() != 0) {
    Serial.println("Failed to init distance sensor");
  }

  // let's not pretend
  distanceSensor.setDistanceModeShort();
  // TODO config window, etc
  // TODO what is reasonable?
  distanceSensor.setTimingBudgetInMs(timingBudgetMs);
  // >= timing budget
  distanceSensor.setIntermeasurementPeriod(timingBudgetMs);
  distanceSensor.startRanging();

  // setup mirror motor
  ocp_Setup(pwmMirrorPin);
}

int inline getRange(void) {
  // make sure we're waiting for a new sample
  while (!distanceSensor.checkForDataReady()) {
    // it seems to get unhappy if we poll too quickly
    delay(1);
  }
  int const spad = distanceSensor.getSignalPerSpad();
  uint8_t const err = distanceSensor.getRangeStatus();
  int const distance = distanceSensor.getDistance();
  
  // check getRangeStatus, return error if failed (0 success)
  if (err != 0) {
    // FIXME rm
    //Serial.println("status err");
    return -1;
  }
  else if (spad < minimumSPAD) {
    return -2;
  }
  return distance;
}

bool rotary_Home(void) {
  homedDurationMs = 0;

  int distance = 0;

  // start rotating at the normal speed
  // take take measurements
  // when measurement drops below close thresh, mark time
  // mark time when measurement exceeds close thresh
  // repeat X times to and print stuff

  Serial.println("Starting to home...");
  digitalWrite(ledPin, LOW);

  ocp_SetDuty(defaultMirrorSpeed);

  // FIXME rm
  Serial.printf("1=%dmm ", getRange());
  Serial.printf("(%d/spad)", distanceSensor.getSignalPerSpad());

  // spin until we start seeing the backstop
  do {
    distance = getRange();
  } while ((distance > closeThreshMm + closeThreshHysteresisMm) ||
           distance < 0);
  int const backstopStart = millis();

  // FIXME rm
  Serial.printf("2=%dmm ", distance);
  Serial.printf("(%d/spad)", distanceSensor.getSignalPerSpad());

  // make sure we wait until there has been at least one more measurement
  distanceSensor.checkForDataReady();

  // spin until we stop seeing the backstop
  do {
    distance = getRange();
  } while ((distance > 0 && (distance <= closeThreshMm + closeThreshHysteresisMm)) ||
           distance < 0);
  int const backstopStop = millis();

  // FIXME rm
  Serial.printf("3=%dmm ", distance);
  Serial.printf("(%d/spad)", distanceSensor.getSignalPerSpad());

  digitalWrite(ledPin, HIGH);

  // TODO filter this
  homedDurationMs = backstopStop - backstopStart;
  Serial.printf(" Backstop was %d ms wide\n", homedDurationMs);

  // TODO calc full duration
  //360*(backstopTimeMs / 70
  Serial.printf("    %dms full duration\n", (unsigned)(360.0 * (float)homedDurationMs / 70.0));
  // at speed 20, budget 20, backstop is 243 or 263ms wide. 360 is 1249 or 1352ms around (say 1300ms)
  // 1300 / (20 + 20) = 32.5 scan cycles/revolution
  // 360/32.5 = 11.1 deg-wide scan sectors
  return true;
}

void rotary_ScanContinuous(void) {
  // TODO run until we think the homing data is bad, then return and implicitly rehome
  // keep all vars local on the stack so they reset
  uint16_t scanData[360] = {};
}

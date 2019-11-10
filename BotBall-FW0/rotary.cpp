#include "rotary.h"
#include "OcPwm.h"
#include "hw.h"

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include <Wire.h>

// number of segments in scan data array
#define NUM_SCAN_SEGMENTS (60)

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
const int timingBudgetGapMs = timingBudgetMs + 4;
const int scanSectorWidthMs = timingBudgetMs + timingBudgetGapMs;

// below this (or low confidence) we consider the backstop
const int closeThreshMm = 40;
const int closeThreshHysteresisMm = 12;

// anything below this is probably bogus (too far, etc)
const int minimumSPAD = 800;

// TODO IIR?
float degreesPerSector = 0;
uint32_t backstopWidthMs = 0;

SFEVL53L1X distanceSensor(Wire1, distanceNShutdown, distanceInt);

void findClossestTarget(int const * const scanArray, int const scanArrayLen, int const numElements);

void rotary_Begin(void) {
  Wire1.begin();
  Wire1.setSDA(distanceSDA);
  Wire1.setSCL(distanceSCL);

  if (distanceSensor.begin() != 0) {
    Serial.println("Failed to init distance sensor");
    digitalWrite(ledRPin, LOW);
  }
  delay(500);

  // in case it was still running
  distanceSensor.stopRanging();

  // let's not pretend
  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(timingBudgetMs);
  // >= timing budget
  distanceSensor.setIntermeasurementPeriod(timingBudgetGapMs);
  // don't set any params after this
  distanceSensor.startRanging();
  delay(100);

  // setup mirror motor
  ocp_Setup(pwmMirrorPin);
}

int inline getRange(void) {
  // this is insane, but if it works...
  static int lastRange = 0;

  // make sure we're waiting for a new sample
  /*
  while (!distanceSensor.checkForDataReady()) {
    // it seems to get unhappy if we poll too quickly
    Serial.print(".");
    delay(1);
  }
  */
  
  int distance;
  do {
    distance = distanceSensor.getDistance();
    //distanceSensor.checkForDataReady()
  } while(distance == lastRange);
  lastRange = distance;
  
  int const spad = distanceSensor.getSignalPerSpad();
  int const sigRate = distanceSensor.getSignalRate();
  int const ambientRate = distanceSensor.getAmbientRate();
  uint8_t const err = distanceSensor.getRangeStatus();
  //int distance = distanceSensor.getDistance();

  // check getRangeStatus, return error if failed (0 success)
  if (err != 0) {
    // FIXME rm
    Serial.println("signal err");
    return -1;
  }
  // FIXME rm
  //Serial.printf("%3d @ %5d spad \t%4d amb \t%5d sig\n", distance, spad, ambientRate, sigRate);

  if (spad < minimumSPAD) {
    return -2;
  }
  return distance;
}

// takes the RAW return from getRange (inc error code!) and tells you if it probably sees the backstop
bool probablySeesBackstop(int const distance) {
  // if it sees something close
  return distance > 0 && (distance <= closeThreshMm + closeThreshHysteresisMm);
  
  /*
  // if it sees something close or is SPAD error
  return (distance > 0 && (distance <= closeThreshMm + closeThreshHysteresisMm)) ||
         distance < 0;
         */
}

bool rotary_Home(void) {
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
  //} while(!probablySeesBackstop(distance));
  int const backstopStart = millis();

  // FIXME rm
  Serial.printf("2=%dmm ", distance);
  Serial.printf("(%d/spad)", distanceSensor.getSignalPerSpad());

  // spin until we stop seeing the backstop
  do {
    distance = getRange();
  } while (probablySeesBackstop(distance));
  int const backstopStop = millis();

  // FIXME rm
  Serial.printf("3=%dmm ", distance);
  Serial.printf("(%d/spad)", distanceSensor.getSignalPerSpad());

  digitalWrite(ledPin, HIGH);

  // TODO filter this
  int const homedDurationMs = backstopStop - backstopStart;
  // useful for sanity checks in runloop
  backstopWidthMs = homedDurationMs;
  Serial.printf(" Backstop was %d ms wide\n", homedDurationMs);

  // 70* was measured in CAD is the amount of the LIDAR's view obscured by the backstop
  float const durationFullRotation = 360.0 * (float)homedDurationMs / 70.0;
  Serial.printf("    %dms full duration -> ", (int)durationFullRotation);

  // at speed 20, budget 20, backstop is 243 or 263ms wide. 360 is 1249 or 1352ms around (say 1300ms)

  // 1300 / (20 + 20) = 32.5 scan cycles/revolution
  // 360/32.5 = 11.1 deg-wide scan sectors
  degreesPerSector = 360.0 / (durationFullRotation / (float)scanSectorWidthMs);
  Serial.printf("~%d deg / sector\n", (int)degreesPerSector);
  return true;
}

void rotary_ScanContinuous(void) {
  // run until we think the homing data is bad, then return and implicitly rehome
  // keep all vars local on the stack so they reset

  // these are degreesPerSector wide
  int scanData[NUM_SCAN_SEGMENTS] = {};
  int currentScanSegment = 0;

  // indicate the main loop is running with the green channel of the RGB LED
  digitalWrite(ledGPin, LOW);

  uint32_t startSeeingBackstopMs = 0, stopSeeingBackstopMs = 0;
  bool seesBackstop = 0;
  int numVeryWrongBackstopWidth = 0;

  while (true) {
    int const distance = getRange();
    // we are OK if distance is an error (that gets fed into probablySeesBackstop)
    if (probablySeesBackstop(distance)) {
      // pseudo-home
      if (!seesBackstop) {
        // transition to seeing backstop
        startSeeingBackstopMs = millis();
        seesBackstop = true;

        // search for a target in scan results
        findClossestTarget(scanData, NUM_SCAN_SEGMENTS, currentScanSegment);
      }
      else {
        // continuing to look at backstop, nothing to do
      }
    }
    else {
      // normal scan
      if (seesBackstop) {
        // transition out of seeing backstop
        stopSeeingBackstopMs = millis();
        seesBackstop = false;

        // guess if this scan was wildly off from expectations
        uint32_t const thisBackstopWidth = stopSeeingBackstopMs - startSeeingBackstopMs;
        if (thisBackstopWidth > backstopWidthMs * 2 ||
            thisBackstopWidth < backstopWidthMs / 2) {
          numVeryWrongBackstopWidth++;
        }
        if (numVeryWrongBackstopWidth > 5) {
          Serial.println("Too many bad backstops");
          // if we have had too many strange scans recently, force a re-home
          digitalWrite(ledGPin, HIGH);
          return;
        }

        // scan was ok, reset the scan slice counter and storage arrays
        memset(scanData, 0, NUM_SCAN_SEGMENTS * sizeof(scanData[0]));
        currentScanSegment = 0;

        // this slice wasn't backstop, so store it
      }

      // normal scan, just store the measurement
      if (currentScanSegment >= NUM_SCAN_SEGMENTS) {
        Serial.printf("Too many scans (%d) for the number of segments\n", currentScanSegment);
        // something has gone wrong, rehome
        digitalWrite(ledGPin, HIGH);
        return;
      }

      // TODO filter?
      scanData[currentScanSegment] = distance;
      currentScanSegment++;
    }

    //delay(timingBudgetGapMs);
  }


  digitalWrite(ledGPin, HIGH);
}

void findClossestTarget(int const * const scanArray, int const scanArrayLen, int const numElements) {
  // TODO real algo

  Serial.printf("Find target %d:\n", numElements);
  for (int i = 0; i < numElements; i++) {
    int const * const r = &scanArray[i];
    Serial.printf("%3d ", *r);
  }
  Serial.println("");
}

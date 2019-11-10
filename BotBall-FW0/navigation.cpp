#include "navigation.h"
#include "hw.h"

#include <Arduino.h>

static const int targetDistanceMm = 50;

static const int targetRelHeadingErrorDeg = 5;

enum TrackingState {
  TS_UNKNOWN,
  TS_NO_TARGET,
  TS_SEEKING,
  TS_ARRIVED,
};

static struct {
  enum TrackingState tracState;

  int targetRangeMm;
  int targetHeadingRel;

  int currentBaseVeloc;
} state = {};

static void programMotors(void);

void nav_Begin(void) {
  memset(&state, 0, sizeof(state));
  state.tracState = TS_NO_TARGET;

  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);
}

// clockwise is positive, in degrees
void nav_SetHeadingOffset(int16_t const headingCCPDeg) {
  state.targetHeadingRel = headingCCPDeg;
  Serial.printf("New rel heading: %d*\n", state.targetHeadingRel);
  programMotors();
}

void nav_SetRange(uint16_t const newRangeMm) {
  state.targetRangeMm = newRangeMm;
  Serial.printf("New range: %dmm", state.targetRangeMm);
  programMotors();
}

// tells us if we are facing directly at the target, and it is within the range
static bool haveArrivedAtTarget(void) {
  return state.targetRangeMm <= targetDistanceMm ||
         abs(state.targetHeadingRel) < targetRelHeadingErrorDeg;
}

static void programMotors(void) {
  // set blue LED for status
  if (state.tracState == TS_ARRIVED) {
    // done!
    digitalWrite(ledBPin, HIGH);
  }
  else if (state.tracState != TS_UNKNOWN) {
    digitalWrite(ledBPin, LOW);
  }

  // if there is an error to target, navigate
  if (haveArrivedAtTarget()) {
    state.tracState = TS_ARRIVED;
    return;
  }
  // else, there is navigation to do

  // adjust base speed based on range
  int rangeDelta = state.targetRangeMm - targetDistanceMm;
  if (rangeDelta > 0) {
    // TODO make this...proportional
    state.currentBaseVeloc = 20;
  }
  else {
    state.currentBaseVeloc = 0;
  }

  int16_t speedA = state.currentBaseVeloc;
  int16_t speedB = state.currentBaseVeloc;

  if (state.targetHeadingRel < 0) {
    speedA -= 5;
    speedB += 5;
  }
  else if (state.targetHeadingRel > 0) {
    speedA += 5;
    speedB -= 5;
  }

  // set directions. A low, B high, is foreward
  if (speedA < 0) {
    digitalWrite(phaseAPin, LOW);
  }
  else {
    digitalWrite(phaseAPin, HIGH);
  }

  if (speedB < 0) {
    digitalWrite(phaseBPin, HIGH);
  }
  else {
    digitalWrite(phaseBPin, LOW);
  }

  // set the actual speed
  analogWrite(pwmAPin, speedA);
  analogWrite(pwmBPin, speedB);
}

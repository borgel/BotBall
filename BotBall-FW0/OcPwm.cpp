#include "OcPwm.h"

#include <stdint.h>
#include <Wiring.h>

#include "IntervalTimer.h"

static IntervalTimer timer;
static uint8_t count;
static volatile uint8_t countCompare;
static int pin;
static int const periodTicks = 100;
static bool paused = false;

static void handlePwmTick(void);

void ocp_Setup(int const newPin, uint8_t const dutyCycle) {
  pin = newPin;
  count = 0;
  countCompare = dutyCycle;

  // 150000 = 0.15s
  //  10000 = 0.01s
  //   1000 = 0.001s
  //    100 = 0.0001s
  // no easy way to pass state ctx to this CB, so don't bother encapsulating
  timer.begin(handlePwmTick, periodTicks);
}

void ocp_Pause(void) {
  //timer.end();
  paused = true;
}
void ocp_Resume(void) {
  count = 0;
  paused = false;
}

// in counts/255
void ocp_SetDuty(uint8_t const newDuty) {
  countCompare = newDuty;
}

static void handlePwmTick(void) {
  if(paused) {
    digitalWriteFast(pin, HIGH);
    pinMode(pin, INPUT);
    return;
  }
  count++;
  if (count > countCompare) {
    // "open drain" hi-z
    pinMode(pin, INPUT);
  }
  else {
    pinMode(pin, OUTPUT);
    digitalWriteFast(pin, LOW);
  }
}

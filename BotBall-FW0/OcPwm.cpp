#include "OcPwm.h"

#include <stdint.h>
#include <Wiring.h>

#include "IntervalTimer.h"

static IntervalTimer timer;
static uint8_t count;
static volatile uint8_t countCompare;
static int pin;
static int const periodTicks = 100;

static void handlePwmTick(void);

void ocp_Setup(int const newPin) {
  pin = newPin;
  count = 0;
  countCompare = 0;

  // 150000 = 0.15s
  //  10000 = 0.01s
  //   1000 = 0.001s
  //    100 = 0.0001s
  // no easy way to pass state ctx to this CB, so don't bother encapsulating
  ocp_Resume();
}

void ocp_Pause(void) {
  timer.end();
}
void ocp_Resume(void) {
  timer.begin(handlePwmTick, periodTicks);
}

// in counts/255
void ocp_SetDuty(uint8_t const newDuty) {
  if (countCompare == 0 && newDuty > 0) {
    // restart timer
    ocp_Resume();
  }
  countCompare = newDuty;

  if (countCompare == 0) {
    // stop timer for now
    ocp_Pause();
  }
}

static void handlePwmTick(void) {
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

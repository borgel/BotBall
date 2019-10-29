#include "OcPwm.h"

#include <stdint.h>
#include <Wiring.h>

#include "IntervalTimer.h"

static IntervalTimer timer;
static uint8_t count;
static volatile uint8_t countCompare;
static int pin;

static void handlePwmTick(void);

void ocp_Setup(int const newPin) {
  pin = newPin;
  count = 0;
  countCompare = 0;

  // 150000 = 0.15s
  //  10000 = 0.01s
  //   1000 = .001s
  // no easy way to pass state ctx to this CB, so don't bother encapsulating
  timer.begin(handlePwmTick, 100);
}

// in counts/255
void ocp_SetDuty(uint8_t const newDuty) {
  countCompare = newDuty;
}

static void handlePwmTick(void) {
  count++;
  if (count > countCompare) {
    // "open drain" hi-z
    pinMode(pin, INPUT);
  }
  else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

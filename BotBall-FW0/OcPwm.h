// a library for doing open drain PWM with normal GPIO

#include <stdint.h>

void ocp_Setup(int const newPin);
void ocp_SetDuty(uint8_t const newDuty);
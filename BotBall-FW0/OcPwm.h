// a library for doing open drain PWM with normal GPIO

#include <stdint.h>

void ocp_Setup(int const newPin, uint8_t const dutyCycle);
void ocp_Pause(void);
void ocp_Resume(void);
void ocp_SetDuty(uint8_t const newDuty);

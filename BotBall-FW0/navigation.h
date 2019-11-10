#pragma once

#include <stdint.h>

void nav_Begin(void);
void nav_SetHeadingOffset(int16_t const headingCCPDeg);
void nav_SetRange(uint16_t const newRangeMm);

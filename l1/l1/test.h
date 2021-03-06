#ifndef TEST_HEAD
#define TEST_HEAD

#include <avr/pgmspace.h>
#include "pololu/orangutan.h"
#include "ohmm/ohmm.h"

// radius of wheel in mm
#define WHEEL_RADIUS 40
// baseline in mm
#define BASELINE 201

void dgvw();
void dsvw(float v, float w);
void dsvl(float v, float l);

#endif

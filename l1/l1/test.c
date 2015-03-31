#include "test.h"

void dgvw()
{
  float l, r, v, w;
  //float *v = (float)malloc(sizeof(float));
  //float *w = (float)malloc(sizeof(float));

  // get left and right speed in rev/sec
  motGetVel(&l, &r);

  float c; //circumference 
  c = 2*M_PI*WHEEL_RADIUS;

  //robot forward velocity in mm/sec (rev/s * circumference)
  v = c * 0.5 * (l + r);

  //robot ccw angular velocity
  //ccw ???
  w = (WHEEL_RADIUS / BASELINE) * (r - l);

  //NOTE must return something
}

void dsvw(float v, float w) {

  float l, r;

  // get r wheel velocity in rev/sec
  // is this rev/sec or mm/sec stil?? mm/sec
  r = (2 * v + BASELINE * w) / (2 * WHEEL_RADIUS);

  // get l wheel velocity in rev/sec
  l = (2 * v - BASELINE * w) / (2 * WHEEL_RADIUS);

  motSetVelCmd(l, r)
}


void dsvl(float v, float l) {
  float w;

  //change v, l - to v w

  dsvw(v, w);
}

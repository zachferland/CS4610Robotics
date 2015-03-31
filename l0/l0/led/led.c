/**
 *  CS4610 Lab 0 LED on demo program.
 **/

/* #include <file.h> searches in system directories only */
/* #include "file.h" searches in local directories, then system directories */
#include "pololu/orangutan.h"

/* uncomment this to use the Pololu LED API, comment it out to directly use
   Special Function Registers */

/* #define USE_POLOLU_LIB */

/**
 * Program entry point.
 **/
int main(/* note, no args on avr */) {

/* turn on red LED */
#ifdef USE_POLOLU_LIB
  print("using Pololu Lib");
  green_led(HIGH); 
#else
  print("using SFRs");
  DDRC |= 1<<4; /* configure port C pin 4 as an output */
  PORTC |= 1<<4; /* set output high */
#endif

  return 0; /* actually just enters infinite loop on avr */
}


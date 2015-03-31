/**
 *  CS4610 Lab 0 adding demo program.
 **/

/* #include <file.h> searches in system directories only */
/* #include "file.h" searches in local directories, then system directories */
#include "pololu/orangutan.h"

/* for printf() */
#include <stdio.h>

/**
 * Program entry point.
 **/
int main(/* note, no args on avr */) {

  /* Without the volatile modifier, the compiler would probably see that the
     values for a and b are known at compile time, and will optimize out code
     to actually add them.  Instead, it will just generate code to store the
     value of their compile-time sum directly in s.  The volatile modifier
     tells the compiler that it could be possible for the values to change
     before they are added by some other means.  That is not really possible
     here, but we use volatile to force the compiler not to optimize out the
     add instruction.  This lets you inspect the generated dissassembly listing
     "add.lss" so you can see how your c code is translated into machine
     instructions. */
	volatile uint8_t a = 7, b = 11;
	volatile uint8_t s = a + b;

	lcd_init_printf();
	printf("%d + %d = %d", a, b, s);

  return 0; /* actually just enters infinite loop on avr */
}


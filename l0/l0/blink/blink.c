/**
 *  CS4610 Lab 0 LED blink demo program.
 **/

/* #include <file.h> searches in system directories only */
/* #include "file.h" searches in local directories, then system directories */
#include "pololu/orangutan.h"

/**
 * Half the blink cycle period.
 **/

// #define BLINK_MS 500

/**
 * Program entry point.
 **/
int main(/* note, no args on avr */) {

  print("blinking is fun"); /* on the lcd */

  red_led(HIGH); /* turn on red LED */

  unsigned long last_toggle = get_ms();

  int blink_ms = 500;  // my definied blink time

  while (1) { /* infinite loop */


    unsigned char button = get_single_debounced_button_release(ANY_BUTTON);
  
 
    if (button & TOP_BUTTON) {
      if (blink_ms > 10) {
        blink_ms = blink_ms - 10;
      }
    } 
      
    if (button & MIDDLE_BUTTON) {
      blink_ms = 500;
    }  
      
    if (button & BOTTOM_BUTTON) {
      blink_ms = blink_ms + 10;
    }  
     

    /* Eventually "now" will reach the maximum value that can be stored in an
       unsigned long integer (unsigned long is 4 bytes with avr-gcc).  Its
       value will cycle around back to zero, and then continue up from there.
       Interestingly, the "diff" calculation will work correctly even when this
       wraparound occurs. */
    unsigned long now = get_ms();

    unsigned long diff = now-last_toggle;

    if (diff > blink_ms) { /* time to toggle ? */

      red_led(TOGGLE); /* yup, toggle */

      last_toggle = now; /* remember when we toggled */

      lcd_goto_xy(0,1); /* column 0, row 1 */
      print_long(diff); print("ms");
    }

  }

  /* return statement is required to compile, but this code will actually never
     get here due to the intentional infinite loop above */

  return 0; /* actually just enters infinite loop on avr */
}


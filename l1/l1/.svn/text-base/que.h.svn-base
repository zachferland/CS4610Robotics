#if !defined(QUE_H) /* protect against multiple inclusion */
#define QUE_H


/** for uint8_t etc. **/
#include <stdint.h>

/**
 * Pointer to a function taking no parameters and returning nothing.
 **/
typedef void (*command_t)(float);

/**
 * Initializes que
**/
void  queInit();

/**
 * Adds a command to the end of the que
 * 
 * command - a command structure of command dt or df
 * arg - a sinle argument to the command
 **/
void queAdd(command_t command, float arg);

/**
 * Runs next command in que
 *
 **/
void queNext();

/**
 * Returns number of commands in the que
 **/
int8_t queCount();

/**
 * Removes command from from front of que
 **/
void queRemove();

/**
 * ReInitializes and empties que
 **/
void queReInit();

/**
 * Checks if que is emtpy
 *
 * returns 1 equivalent to true or 0 equivalent to false
 **/
int8_t queEmptyHuh();

#endif /* protect against multiple inclusion */
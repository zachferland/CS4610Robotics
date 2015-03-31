
#include <avr/pgmspace.h>

#include "que.h"

#include <stdio.h>
#include <string.h>

#include "ohmm/mot.h"
#include "ohmm/cmd.h"
#include "ohmm/task.h"
#include "ohmm/hlp.h"

/**
 * All info about a command in que
 **/
typedef struct {

  /**
   * A command function, or 0 if this is an empty task slot.
   **/
  volatile command_t command;

  /**
   * Argument for command df or dt
   **/
  volatile float arg;

} command_info_t;

/**
 * The que array of commands
 **/
static 	command_info_t que[32];

/**
 * Keeps track of next available command
 **/
static volatile int8_t first_que = 0;

/**
 * Keeps track track of position of the end of the que
 **/
static volatile int8_t end_que = 0;


void queInit() {
  memset(que, 0, sizeof(que)); /* clear the que array */
}

void queAdd(command_t command, float arg){

    command_info_t *ci = &(que[end_que]);

    ci->command = command;
    ci->arg = arg;
 	
 	//Circular Que
    (end_que++) % 32;

    /**
	 * Should have implemented a limit if not array slots available
 	**/
}

void queNext(){
	if (!(que[first_que].command)) return;
	que[first_que].command(que[first_que].arg);
}


int8_t queCount() {
  return end_que - first_que;
}

void queRemove() {
	command_info_t *ci = &(que[first_que]);

    ci->command = 0;
    ci->arg = 0;
    
    //Circular Que
    (first_que++) % 32;
}

void queReInit() {
	memset(que, 0, sizeof(que)); /* clear the que array */
}

int8_t queEmptyHuh() {
  if (end_que = first_que) return 1;
  else return 0;
}
// Blinking a 12V lamp using a transistor

//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

int main(void)
{
  DDRA = 0xFF; // PORTA in output mode
  PORTA = 0;

  while(1) //main loop
  {
    // Turn PA5 on and off
    _delay_ms(500);
    PORTA |= (1<<PA5); 
    _delay_ms(500); 
    PORTA &= ~(1<<PA5);
  }
  return(1);
} //end main 

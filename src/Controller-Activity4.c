//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

// ACTIVITY 4: Demonstrate 8 LEDs blinking in sequence

int main(void)
{
  DDRA = 0xFF;//put PORTA into output mode
  PORTA = 0; 
  while(1)//main loop
  {
    for (int i = 0; i < 8; i++)
    {
      PORTA = (1<<i);
      _delay_ms(500);
    }
  }
  return(1);
}//end main

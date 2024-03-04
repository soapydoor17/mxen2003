//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

// ACTIVITY 5: Demonstrate the blinking sequence on a separate PORT

int main(void)
{
  DDRC = 0xFF; // NOTE - DDR must be changed as well as ports!
  PORTC = 0; 
  while(1)//main loop
  {
    for (int i = 0; i < 8; i++)
    {
      PORTC = (1<<i)
      _delay_ms(500);
    }
  }
  return(1);
}//end main

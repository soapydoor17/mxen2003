//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Robot.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file

int main(void)
{
  DDRA = 0xFF;//put PORTA into output mode
  PORTA = 0; 
  while(1)//main loop
  {
    _delay_ms(500);     //500 millisecond delay
    PORTA |= (1<<PA2);  // note here PA3 is just an alias for the number 3
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA
    _delay_ms(500); 
    PORTA &= ~(1<<PA2); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA
  }
  return(1);
}//end main 

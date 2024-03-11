// Push button to toggle state of 12 V lamp using an interrupt

//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file

int main(void)
{
  // interrupt setup
  cli();             // Clear interrupts
  DDRD &= ~(1<<PD0); // INT0 input mode
  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00); // INT0 set falling edge trigger
  EIMSK |= (1<<INT0);   // INT0 enable
  sei();

  while(1) //main loop
  {
    
  }
  return(1);
} //end main 

ISR(INT0_vect)
{
  // interrupt code here
}

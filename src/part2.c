// Push button to toggle state of 12 V lamp using an interrupt

//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
//#include "../lib/adc/adc.h" // minimal adc lib
#include <aur/interupt.h>
#define DEBOUNCE_PERIOD 100

//static function prototypes, functions only called in this file

int main(void)
{
  // interrupt setup
  milliseconds_init();
  cli();             // Clear interrupts
  DDRD &= ~(1<<PD0); // INT0 input mode
  PORTD |= (1<<PD0);
  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00); // INT0 set falling edge trigger
  EIMSK |= (1<<INT0);   // INT0 enable
  DDRA = 0xFF;
  PORTA = 0;
  
  sei();

  while(1) //main loop
  {
  }
  return(1);
} //end main 

ISR(INT0_vect)
{
  uint32_currTime = milliseconds_now();
  static uint32_t prevTime = 0;
  if((currTime-prevTime) > DEBOUNCE_PERIOD)
  {
    PORTA ^= (1<<PA1);
    prevTime = currTime;
  }
}

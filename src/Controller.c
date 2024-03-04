//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file

int main(void)
{
  DDRA = 0;          // put PORTA into input mode
  DDRA |= (1<<PA3);  // PA3 output mode
  uint8_t a = PINA;  // Digital input for Port A
  PORTA = 0;         // All digital outputs of A are LOW
  PORTA |= (1<<PA7); // enable internal pullup PA7 (as it is in input mode)

  adc_init(); // initialises ADC
  _delay_ms(20);

  uint16_t adcVal = 0; // variable which input will be assigned to

  while(1) //main loop
  {
    if (!(PINA & (1<<PA7))) // if button is pressed (LOW is pressed)
    {
      PORTA |= (1<<PA3);
    }
    else{
      PORTA &= ~(1<<PA3);
    }
    adcVal = adc_read(3); // read voltage at ADC3 // returns 10-bit value
  }
  return(1);
} //end main 

//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file

int main(void)
{
  adc_init(); // initialises ADC
  _delay_ms(20);

  DDRC = 0xFF;       // put PORTC into output mode
  DDRF = 0;          // put PORTF into input mode
  PORTC = 0;         // All digital outputs of C are LOW
  PORTF |= (1<<PF7);  // pull up resistor for button
  uint16_t reading = 0;  // where joystick reading is assigned

  while(1) //main loop
  {
    if (!(PINF & (1<<PF7))) // if button is pressed read PF5 else read PF3
    {
      reading = adc_read(5);
    }
    else{
      reading = adc_read(3);
    }

    reading = reading >> 2; // most significant bits shift down so its 8 bit
    PORTC = reading;        // display 8 bit reading on LEDs

  }
  return(1);
} //end main 

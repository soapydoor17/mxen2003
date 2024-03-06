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
  uint16_t reading = 0;  // where joystick reading is assigned
  uint16_t MinVoltage = 492; //convert voltages to 10bit 
  uint16_t MaxVoltage = 450; //convert voltages to 10bit
  uint16_t ledOutput = 0;
    
  while(1) //main loop
  {
    reading = adc_read(5);
    ledOutput = (MinVoltage - reading) / (MinVoltage - MaxVoltage) * 255; // convert reading to 8bit
    PORTC = (uint8_t) ~ledOutput;        // display 8 bit reading on LEDs
    
  }
  return(1);
} //end main 

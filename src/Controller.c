//Author: Tristan Davies
//Created: Jan 2024
//An example of LCD commands

//include this .c file's header file
#include "Controller.h"
#include <avr/interrupt.h>

#define DEBOUNCE_PERIOD 100

// Global integer set up (needs to be accessed by interrupts)
bool freeze = 0;			// when true, live feed is frozen
uint16_t currVal = 0;
uint16_t cmVal = 0;

//static function prototypes, functions only called in this file

int main(void)
{
	// initialisation section, runs once
	adc_init(); // initialse ADC
	lcd_init(); // initialise LCD
	_delay_ms(20);

	// Port Initialising
	DDRF = 0;			// PortF input for range sensor
	DDRD = 0;			// PortD input for buttons
  PORTD = 0;
	PORTD |=(1<<PD0);	// pull-up resistors for button
	
	// Interrupt Initialsing
	EICRA |= (1<<ISC01);
	EICRA &= ~(1<<ISC00); // INT0 set falling edge trigger
	EIMSK |= (1<<INT0);   // INT0 enable
	

	//variable declarations
  // declare and initialise strings for LCD
	char line2_string[33] = {0};
	
	sei(); // set up interrupts

	//main loop
	while(1)
	{	
		// Live Feed
		lcd_home();
		if (freeze == 0)
		{
			// if not frozen, update the current value
			currVal =  adc_read(0);
      cmVal = 7000/currVal - 6;
			lcd_puts("Current Value:");
		}
		else
		{
		}
		lcd_goto(0x40);
		sprintf(line2_string, "%5u cm", cmVal);
		lcd_puts(line2_string);

  } //end main
  return(1);
}

// Freeze Toggle
ISR(INT0_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) < DEBOUNCE_PERIOD)
	{
		freeze ^= 1;
    prevTime = currTime;
	}
}

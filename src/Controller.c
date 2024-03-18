//Author: Tristan Davies
//Created: Jan 2024
//An example of LCD commands

//include this .c file's header file
#include "Controller.h"
#define DEBOUNCE_PERIOD 100

// Global integer set up (needs to be accessed by interrupts)
bool displayMode = false;		// false is live feed. true is saved values
bool freeze = false;			// when true, live feed is frozen
uint16_t currVal = 0;
uint16_t savedVal1 = 0;			// variables to be saved
uint16_t savedVal2 = 0;

//static function prototypes, functions only called in this file

int main(void)
{
	// initialisation section, runs once
	adc_init(); // initialse ADC
	lcd_init(); // initialise LCD
	_delay_ms(20);

	// Port Initialising
	DDRF = 0;			// PortF input for range sensor
	DDRD = 0;			// PortA input for buttons
	PORTD |=(1<<PD0);	// pull-up resistors for buttons
	PORTD |=(1<<PD1);
	PORTD |=(1<<PD2);
	PORTD |=(1<<PD3);	
	
	// Interrupt Initialsing
	DDRD &= ~(1<<PD0); // INT0 input mode
	EICRA |= (1<<ISC01);
	EICRA &= ~(1<<ISC00); // INT0 set falling edge trigger
	EIMSK |= (1<<INT0);   // INT0 enable
	EICRA |= (1<<ISC11);
	EICRA &= ~(1<<ISC10); // INT1 set falling edge trigger
	EIMSK |= (1<<INT1);   // INT1 enable
	EICRA |= (1<<ISC21);
	EICRA &= ~(1<<ISC20); // INT2 set falling edge trigger
	EIMSK |= (1<<INT2);   // INT2 enable
	EICRA |= (1<<ISC31);
	EICRA &= ~(1<<ISC30); // INT3 set falling edge trigger
	EIMSK |= (1<<INT3);   // INT3 enable

	//variable declarations
	char line1_string[33] = {0};	// declare and initialise strings for LCD
	char line2_string[33] = {0};
	
	sei(); // set up interrupts

	//main loop
	while(1)
	{	
		// Live Feed
		if (displayMode == false)
		{
			lcd_home();
			if (freeze == false)
			{
				// if not frozen, update the current value
				currVal =  adc_read(0);
				lcd_puts("Current Value:");
			}
			else
			{
				lcd_puts("Current Value:       (FROZEN)");
			}

			lcd_goto(0x40);
			sprintf(line2_string, "%u bits", currVal);
			lcd_puts(line2_string);
			lcd_clrscr();
		}
		else
		{
			// Line 1
			lcd_home();
			sprintf(line1_string, "Value 1: %u",savedVal1);
			lcd_puts(line1_string);

			// Line 2
			lcd_goto(0x40);
			sprintf(line2_string, "Value 2: %u", savedVal2);
			lcd_puts(line2_string);

			lcd_clrscr();
		}
	
	}
	return(1);
} //end main

// Freeze Toggle
ISR(INT0_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) > DEBOUNCE_PERIOD)
	{
		if (displayMode == false)
		{
			freeze ^= (1>>freeze);
		}
	}
}

// Update saveVal1
ISR(INT1_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) > DEBOUNCE_PERIOD)
	{
		if (displayMode == false)
		{
			savedVal1 = currVal;
		}
	}
}

// Update saveVal2
ISR(INT2_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) > DEBOUNCE_PERIOD)
	{
		if (displayMode == false)
		{
			savedVal2 = currVal;
		}
	}
}

// Toggle view
ISR(INT3_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) > DEBOUNCE_PERIOD)
	{
		displayMode ^= (1>>displayMode);
	}
}

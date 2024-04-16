
//include this .c file's header file
#include "Robot.h"
#include <avr/interrupt.h>
#include "../lib/adc/adc.h" // minimal adc lib


#define DEBOUNCE_PERIOD 100

// Global integer set up (needs to be accessed by interrupts)
uint16_t currVal = 0;
uint16_t cmVal = 0;

//static function prototypes, functions only called in this file

int main(void)
{
	// initialisation section, runs once
	adc_init(); // initialse ADC
	serial0_init(); 	// terminal communication with PC
	serial2_init();	
  
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
		// Reading Range sensor - cmVal gives range in cm
		currVal =  adc_read(0);
    cmVal = 7000/currVal - 6; //!MAY NEED TO BE RECALLIBRATED - USE WEEK 5 CODE TO DO SO!

    // Transmitting 
		current_ms = milliseconds_now();
	
	//sending section
	if( (current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
		last_send_ms = current_ms;
		serial2_write_byte(255); //send start byte
		serial2_write_byte(1); //send test value for first parameter
		serial2_write_byte(2); //send test value for second parameter
		serial2_write_byte(3); //send test value for third parameter
		serial2_write_byte(4); //send test value for fourth parameter
		serial2_write_byte(5); //send test value for fifth parameter
		serial2_write_byte(254); //send stop byte
	}


  } //end main
  return(1);
}

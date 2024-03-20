//week5 lab work
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file
bool timer_start = 1;
uint16_t timer_val = 0;

int main(void)
{
  
  cli();
  adc_init(); // initialse ADC
  lcd_init(); // initialise LCD
  _delay_ms(20);

	//button init stuff
  DDRD = 0;			// PortD input for button
  PORTD = 0;  // PinD0 for button
  PORTD |=(1<<PD0);	// pull-up resistors for button

	//LCD init stuff
  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00); // INT0 set falling edge trigger
  EIMSK |= (1<<INT0);   // INT0 enable
	


  // declare and initialise strings for LCD
  char line2_string[33] = {0};

  //timer init stuff
	
  TTCR1A = 0;
  TTCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS10);  //timer set for mode 12 (CTC) AND prescaler of 1
  TCNT1 = 0;
  ICR1 = 1000;  //sets top value NEEDS TO BE CALCULATED
  TIMSK1 = (1<<ICIE1);  //capture interupt for timer 1
  sei();
  
  while(1) //main loop
  {
    if timer_start == 1;
    {
		  //LCD displays updating time
	    lcd_home();
	    lcd_puts("TIMER (s):");
	    
	    timer_val = TCNT1; // updates timer_val (prob have to update to make readable)
    }
    else
	  {
	    lcd_home();
	    lcd_puts("TIMER||STOP");
	    OC1A = 0;
   

  	  }
    lcd_goto(0x40);
    sprintf(line2_string, "%u", timer_val);
  return(1);
} //end main 

ISR(TIMER1_CAPT_vect)
{
  //interrupt increment
}


//stop start toggle
ISR(INT0_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) < DEBOUNCE_PERIOD)
	{
		timer_start ^= 1;
     		prevTime = currTime;
	}
}

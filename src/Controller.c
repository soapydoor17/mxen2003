//Example ATmega2560 
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#define DEBOUNCE_PERIOD 100

//static function prototypes, functions only called in this file

// Global Variables

int main(void)
{
  cli();
	

  // Port Initialising
  DDRC = 0;                        // put PORTC into input mode
  DDRB |= (1<<PB5);		//pinB5 output mode


  // Timer Interrups
  TCCR1A = (1<<COM1A1);
  TCCR1B |= (1<<WGM13) | (1<<CS11);
  TCNT1 = 0;
  ICR1 = 20000;

  uint16_t compVal = 10000; // need to calc
  OCR1A = compVal;
  
  TIMSK1 = (1<<ICIE1);

  // library initialising
  adc_init();
  //lcd_init();
  _delay_ms(20);

  sei();

  while(1) //main loop
  {
  }
  return(1);
} //end main 



// Timer Interrupt
ISR(TIMER1_CAPT_vect)
{

}

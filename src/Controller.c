//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#define DEBOUNCE_PERIOD 100

//static function prototypes, functions only called in this file

// Global Variables
volatile uint16_t t_millis = 0;
volatile uint16_t t_secs = 0;
volatile uint16_t t_mins = 0;

int main(void)
{
  cli();
	
  // Variable Initialising
  char line1_string[33] ={0};

  // Port Initialising
  DDRD = 0;                        // put PORTD into input mode
  PORTD |= (1<<PD0) | (1<<PD1);    // enable pull-up for PD0 and PD1

  // Button Interrupts
  EICRA |= (1<<ISC01);
  EICRA &= ~(1<<ISC00);            // INT0 to falling edge trigger
  EIMSK |= (1<<INT0);              // enable INT0
  EICRA |= (1<<ISC11);
  EICRA &= ~(1<<ISC10);            // INT1 to falling edge trigger
  EIMSK |= (1<<INT1);              // enable INT1

  // Timer Interrups
  TCCR1A = 0;
  TCCR1B |= (1<<WGM12) | (1<<CS11)|(1<<CS10);;
  TCNT1 = 0;
  OCR1A = 19999;
  TIMSK1 = (1<<OCIE1A);

  // library initialising
  
  lcd_init();
  _delay_ms(20);

  sei();

  while(1) //main loop
  {
    sprintf(line1_string, "%02u : %02u . %02u", t_mins, t_secs, t_millis);
    lcd_home();
    lcd_puts(line1_string);
  }
  return(1);
} //end main 

// BUTTON 1: Start/Stop
ISR(INT0_vect)
{
  uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) < DEBOUNCE_PERIOD)
	{
		TIMSK1 ^= (1<<OCIE1A);
    prevTime = currTime;
	}
}

// BUTTON 2: Reset
ISR(INT1_vect)
{
  uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if((currTime-prevTime) < DEBOUNCE_PERIOD)
	{
		TCNT1 = 0;
    t_millis = 0;
    t_secs = 0;
    t_mins = 0;
	}
}

// Timer Interrupt
ISR(TIMER1_COMPA_vect)
{
  t_millis += 1;
  if (t_millis > 99)
  {
    t_secs += 1;
    t_millis =0;
  }
  if (t_secs > 59)
  {
    t_mins += 1;
    t_secs =0;
  }
}

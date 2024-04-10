//Example ATmega2560 
//File: ATmega2560Project.c
//An example file for second year mechatronics project

// PROGRAM FOR CREATING A PWM SIGNAL TO DISPLAY ON OSCILLOSCOPE

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

// Global Variables

int main(void)
{
  cli();
  // Variable Declaration
  uint16_t compVal1 = 1000;    // set duty cycle2
  uint16_t compVal2 = 1000;
  uint16_t x_reading = 0;     // initialise joystick readings
  uint16_t y_reading = 0;

  // Port Initialising
  DDRB |= (1<<PB5);         // PORT B Output Mode for Servo Motor 1
  DDRE |= (1<<PE3);         // PORT E Output Mode for Servo Motor 2
  DDRF |= 0;                // PORT F Input Mode for Joy Sticks
  OCR1A = compVal1;         // Initialise comparison values for servos
  OCR3A = compVal2;
  
  // Timer Setup
  // PWM Requirement: 50Hz, 1000-2000us pulse duration
  TCCR1A |= (1<<COM1A1);                 // enable PWM on OC1A
  TCCR1B |= (1<<WGM13) | (1<<CS11);      // set Wave Generation Mode 8 and prescaler of ##
  TCNT1 = 0;
  ICR1 = 20000;                // top value
  TIMSK1 |= (1<<ICIE1);        // enables input capture interrupt

  TCCR3A |= (1<<COM3A1);                 // enable PWM on OC1A
  TCCR3B |= (1<<WGM33) | (1<<CS31);      // set Wave Generation Mode 8 and prescaler of ##
  TCNT3 = 0;
  ICR3 = 20000;                // top value
  TIMSK3 |= (1<<ICIE3);        // enables input capture interrupt

  // library initialising
  adc_init();
  _delay_ms(20);


  while(1) //main loop
  {
    x_reading = adc_read(PF1);                              // Read from joystick
    y_reading = adc_read(PF2);

    compVal1 = (x_reading) * (1000) / (1023) + 1000;        // Convert readings
    compVal2 = (y_reading) * (1000) / (1023) + 1000;

    OCR1A = compVal1;           // Update timer comp vals
    OCR3A = compVal2;
  }
  return(1);
} //end main 


//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file
volatile uint8_t dataByte1=0, dataByte2=0;
volatile bool new_message_received_flag=false;

int main(void)
{
  // initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
  milliseconds_init();
  _delay_ms(20);
	
  // variables
  uint16_t fc=0, rc=0;              // Joystick readings received from controller
  static int16_t lm=0, rm =0;       // Speed and direction of motors

  DDRA |= (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3);      // put PORTA into output mode for motors
  PORTA = 0;

  DDRE |= (1<<PE3)|(1<<PE4);        // PORTE output mode for motors
  OCR3A = 8000;
  OCR3B = 8000;
  TCCR3A |= (1<<COM3A1) | (1<<COM3B1);    // OC3A and OC3B clears on compare match
  TCCR3B |= (1<<WGM33) | (1<<CS31);       // WGM 8 and Prescaler of 8
  ICR3 = 10000;                           // TOP is 16000

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

  sei();

  while(1)//main loop
  {
    if (new_message_received_flag)
    {
      fc=dataByte1, rc=dataByte2;

      lm = fc + rc - 253;
      rm = fc - rc;

      OCR3A = (int32_t)abs(lm) * 10000 / 126;
      OCR3B = (int32_t)abs(rm) * 10000 / 126;

      if (lm>=0)
      {
        // left motor goes forward
        PORTA |= (1<<PA0);
        PORTA &= ~(1<<PA1);
      }
      else
      {
        // left motor goes backward
        PORTA &= ~(1<<PA0);
        PORTA |= (1<<PA1);
      }

      if (rm>=0)
      {
        // right motor goes forward
        PORTA |= (1<<PA2);
        PORTA &= ~(1<<PA3);
      }
      else
      {
        // right motor goes backward
        PORTA &= ~(1<<PA2);
        PORTA |= (1<<PA3);
      }

      new_message_received_flag = false;
    }
  }
  return(1);
}//end main 

ISR(USART2_RX_vect)
{
  static uint8_t recvByte1=0, recvByte2=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			
			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}

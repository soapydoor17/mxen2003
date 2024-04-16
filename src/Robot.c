
//include this .c file's header file
#include "Robot.h"
#include <avr/interrupt.h>
#include "../lib/adc/adc.h" // minimal adc lib

// Global integer set up (needs to be accessed by interrupts)
uint16_t currVal = 0;
uint16_t cmVal = 0;
//keeps track of time since last send
uint32_t current_ms = 0;
uint32_t last_send_ms = 0;

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;

int main(void)
{
	// initialisation section, runs once
	adc_init(); // initialse ADC
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// terminal communication with Controller.c
  
	_delay_ms(20);

	// Port Initialising
	DDRF = 0;			// PortF input for range sensor
	
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

	//if a new byte has been received
	if(new_message_received_flag) 
	{
		// now that a full message has been received, we can process the whole message
		// the code in this section will implement the result of your message
		sprintf(serial_string, "received: 1:%4d, 2:%4d , 3:%4d , 4:%4d \n", dataByte1, dataByte2, dataByte3, dataByte4);
		serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received

		new_message_received_flag=false;	// set the flag back to false
	}

  } //end main
  return(1);
}

ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
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
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			
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

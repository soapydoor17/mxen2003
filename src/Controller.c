
//include this .c file's header file
#include "Controller.h"
#include <avr/interrupt.h>
#include "../lib/adc/adc.h" // minimal adc lib

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;

int main(void)
{
	// initialisation section, runs once
	adc_init(); // initialse ADC
	lcd_init(); // initialise LCD
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// terminal communication with Controller.c
  
	_delay_ms(20);

	//joystick setup
	DDRF = 0;                        // put PORTF into input mode
	uint16_t reading = 0;  // where joystick reading is assigned
	uint16_t compVal = 10000; // calc compval (10bit)
	uint8_t compValSEND = 1000; //sent compval (8bit)

	//lcd/range sensor setup	
	uint16_t cmVal = 0;	//CM Value used (10bit)
	uint8_t cmValREC = 0;	//CM Value recieved (8bit)
	char line2_string[33] = {0}; 	// declare and initialise strings for LCD


	//keeps track of time since last send
	uint32_t current_ms = 0;
	uint32_t last_send_ms = 0;


	
	
	sei(); // set up interrupts

	
	while(1)
	{	
	//joystick code
	reading = adc_read(5);	//takes joystick reading from pin F5
	compVal = reading*1000/1023 + 20000; //converts reading to a ratio of total possible power.  
	
	compValSEND = compVal >>2;


    	// Transmitting 
	current_ms = milliseconds_now();
	
	//sending section
	if( (current_ms-last_send_ms) >= 100) //sending rate controlled here
	{
		last_send_ms = current_ms;
		if (sendDataByte1>253) //Causes byte 1 to wrap back to 0 when exceeding 253
		{compValSEND = 0;}
		
		serial2_write_byte(255); //send start byte
		serial2_write_byte(compValSEND); //send byte value
		serial2_write_byte(254); //send stop byte
	}

	//if a new byte has been received
	if(new_message_received_flag) 
	{
		// now that a full message has been received, we can process the whole message
		// the code in this section will implement the result of your message
		sprintf(serial_string, "received: 1:%4d\n", dataByte1);
		serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
		cmValREC= dataByte1;
		new_message_received_flag=false;	// set the flag back to false
	}

	//LCD Code here
	cmVal = cmValREC << 2;
	// Live Feed
	lcd_home();
	lcd_puts("Current Value:");
	lcd_goto(0x40);
	sprintf(line2_string, "%5u cm", cmVal);
	lcd_puts(line2_string);

	

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

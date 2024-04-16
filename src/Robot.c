//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file


//file scope variables
// ** static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;


int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino
	
	uint8_t sendDataByte1=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send

  uint16_t rsVal = 0;       // range sensor value
  uint16_t x_reading=0, y_reading=0;  // reading of joysticks to be sent to robot
  uint16_t compVal1 = 1000, compVal2 = 1000;      // comparison values for PWM waves

  DDRF = 0;               // set PORTF into input mode for range sensor

  // setting up pwm waves n shiz
  DDRB |= (1<<PB5)|(1<<PB6);  // output mode for PORTB pins 5 and 6 (servos)
  OCR1A = compVal1;           // output comparison initialisation
  OCR1B = compVal2;
  TCCR1A |= (1<<COM1A1) | (1<<COM1B1); // OC1A and OC1B clear on upcount and set on downcount
  TCCR1B |= (1<<WGM13) | (1<<CS11);
  TCNT1 = 0;
  ICR1 = 20000;
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
  adc_init();
  _delay_ms(20);
	sei();
	
	while(1)
	{
    // read sensor and convert for sending data to controller

		current_ms = milliseconds_now();
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
    {
			rsVal = adc_read(0);
      sendDataByte1 = rsVal * 253 / 1023;
			
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
      x_reading = dataByte1 *1023/253;
      y_reading = dataByte2 *1023/253;

      compVal1 = x_reading + 1000;
      compVal2 = y_reading + 1000;

      OCR1A = compVal1;
      OCR1B = compVal2;

			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main


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
		case 3: //waiting for stop byte
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
 
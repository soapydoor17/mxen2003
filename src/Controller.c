//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0;		                // data bytes received
volatile bool new_message_received_flag=false;

int main(void)
{
  serial0_init(); 	// terminal communication with PC
  serial2_init();		// microcontroller communication to/from another Arduino
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  uint8_t sendDataByte1=0, sendDataByte2=0;		// data bytes sent
  uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
  uint16_t rsVal=0, cmVal=0, rsVal2=0, cmVal2=0, rsVal3=0, cmVal3=0;          // current value of Range Sensor (received from robot), and cm version
  uint16_t x_reading=0, y_reading=0;  // reading of joysticks to be sent to robot
  char serial0_sting[16] = {0};

  DDRF = 0;          // put PORTF into input mode for joysticks

  UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
  sei();

  while(1) //main loop
  {
    // Sending information
    current_ms = milliseconds_now();
    if(current_ms-last_send_ms >= 100)
    {
      // Read joysticks and convert for sending
      y_reading = adc_read(0);
      x_reading = adc_read(1);

      sendDataByte1 = y_reading / 4;
      if (sendDataByte1 > 253)
      {
        sendDataByte1 = 253;
      }

      sendDataByte2 = x_reading / 4;
      if (sendDataByte2 > 253)
      {
        sendDataByte2 = 253;
      }

      last_send_ms = current_ms;
      serial2_write_byte(0xFF);
      serial2_write_byte(sendDataByte1);
      serial2_write_byte(sendDataByte2);
      serial2_write_byte(0xFE);
    }

  //recinfo
  if(new_message_received_flag) 
{
  // Convert recieved data
    rsVal = (dataByte1 * 4);
    rsVal2 = (dataByte2 * 4);
    rsVal3 = (dataByte3 * 4);
    cmVal = 7000/rsVal - 6;
    cmVal2 = 7000/rsVal2 - 6;
    cmVal3 = 7000/rsVal3 - 6;
    lcd_goto(0x40);
     sprintf(serial0_sting, "%4u; %4u; %4u", cmVal, cmVal2, cmVal3);
     serial0_print_string(serial0_sting);  // print the received bytes to the USB serial to make sure the right messages are received
  }
  return(1);
} //end main 

ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0;		                                    // data bytes received
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
		case 2: //waiting for first parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for first parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			
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

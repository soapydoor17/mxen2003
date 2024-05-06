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
  cli();
  serial0_init(); 	// terminal communication with PC
  serial2_init();		// microcontroller communication to/from another Arduino
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0;		// data bytes sent
  uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
  uint16_t rsValL=0, cmValL=0, rsValF=0, cmValF=0, rsValR=0, cmValR=0;          // current value of Range Sensor (received from robot), and cm version
  uint16_t xr_reading=0, yr_reading=0, xl_reading=0;  // reading of joysticks to be sent to robot: r - right joystick controls motor; l - left joystick controls servo
  char serial0_sting[100] = {0};

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
      yr_reading = adc_read(0);
      xr_reading = adc_read(1);
      xl_reading = adc_read(14);

      sendDataByte1 = yr_reading / 4;
      if (sendDataByte1 > 253)
      {
        sendDataByte1 = 253;
      }

      sendDataByte2 = xr_reading / 4;
      if (sendDataByte2 > 253)
      {
        sendDataByte2 = 253;
      }

      sendDataByte3 = xl_reading / 4;
      if (sendDataByte3 > 253)
      {
        sendDataByte3 = 253;
      }

      last_send_ms = current_ms;
      serial2_write_byte(0xFF);
      serial2_write_byte(sendDataByte1);
      serial2_write_byte(sendDataByte2);
      serial2_write_byte(sendDataByte3);
      serial2_write_byte(0xFE);
    }

  //recinfo
  if(new_message_received_flag) 
  {
    // Convert recieved data
      rsValL = (dataByte1 * 4);
      rsValF = (dataByte2 * 4);
      rsValR = (dataByte3 * 4);
      cmValL = 7000/rsValL - 6;
      cmValF = 7000/rsValF - 6;
      cmValR = 7000/rsValR - 6;
      
      new_message_received_flag = false;
  }
  sprintf(serial0_sting, "LEFT: %5ucm     FRONT: %5ucm     RIGHT: %5ucm", cmValL, cmValF, cmValR);
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

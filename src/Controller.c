//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"
#include "../lib/adc/adc.h" // minimal adc lib

//static function prototypes, functions only called in this file

int main(void)
{
  serial0_init(); 	// terminal communication with PC
  serial2_init();		// microcontroller communication to/from another Arduino
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  uint8_t sendDataByte1=0, sendDataByte2=0;		// data bytes sent
  uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send

  uint16_t x_reading=0, y_reading=0;  // reading of joysticks to be sent to robot

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

  }
  return(1);
} //end main 

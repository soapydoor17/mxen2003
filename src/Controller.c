//include this .c file's header file
#include "Controller.h"
#define DEBOUNCE_PERIOD 100
//static function prototypes, functions only called in this file


//file scope variables
// static char serial_string[200] = {0};
volatile uint8_t dataByte1=0;		                // data bytes received
volatile bool new_message_received_flag=false;
volatile bool freeze = 0;                       // for LCD screen controlled by interrupt


int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
  lcd_init();
  adc_init();
  _delay_ms(20);
	// or loopback communication to same Arduino
	
  // variables
	uint8_t sendDataByte1=0, sendDataByte2=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send

  uint16_t rsVal=0, cmVal=0;          // current value of Range Sensor (received from robot), and cm version
  uint16_t x_reading=0, y_reading=0;  // reading of joysticks to be sent to robot
  char lcd_string[16] = {0};          // string to print to LCD

  DDRD = 0;       // Button input mode with pullup resistor
  PORTD = (1<<PD2);
  DDRF = 0;       // Joysticks in input mode

  EICRA |= (1<<ISC21); // Falling edge trigger for button
  EICRA &= ~(1<<ISC20); 
  EIMSK = (1<<INT2);

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	sei();
	
	while(1)
	{
		current_ms = milliseconds_now();
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
      // Reading joysticks and convert for sending data
      x_reading = adc_read(0);
      y_reading = adc_read(1);

      sendDataByte1 = x_reading *253/1023;
      sendDataByte2 = y_reading *253/1023;

      last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
      // Convert recieved data
      rsVal = dataByte1 * (1023) / (253);

      // Display value on LCD
      lcd_home();
      if (freeze == 0)
      {
        cmVal = 7000/rsVal - 6;
        lcd_puts("Current Value:");
      }
      lcd_goto(0x40);
      sprintf(lcd_string, "%5u cm", cmVal);
      lcd_puts(lcd_string);

      new_message_received_flag = false;
		}
	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0;		                                    // data bytes received
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
		case 2: //waiting for stop byte
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

ISR(INT2_vect)
{
  uint32_t currTime = milliseconds_now();
  static uint32_t prevTime = 0;
  if ((currTime - prevTime) < DEBOUNCE_PERIOD)
  {
    freeze ^= 1;
    prevTime = currTime;
  }
}

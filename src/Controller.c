// MXEN2003 Robot Project Code
// Controller.c - code uploaded to the controller system that sends and receives data from the robot system

#include "Controller.h"
#define DEBOUNCE_PERIOD 200

volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0;   // Data bytes received from the robot
volatile bool new_message_received_flag=false;            // Triggered when all data has been receieved from the robot
volatile bool automode=0;                                 // Determines if robot is in autonomous mode, as triggered by button on controller

/***********************************************************************
MAIN FUNCTION
  Takes inputs from joysticks and buttons, and sends data to robot
  Displays readings from the robot's range sensors on the LCD
Input - None
Output - None
************************************************************************/
int main(void)
{
  cli();    // Disable interrupts

  // Initialise wireless serial communication and libraries for
  // millisecond timer, ADC readings, and LCD display
  serial2_init();
  milliseconds_init();
  adc_init();
  lcd_init();
  _delay_ms(20);

  // Variable Initialisation
  uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// Data bytes sent to robot
  uint32_t current_ms=0, last_send_ms=0;			                                  // Values used for timing the serial send
  uint16_t cmValF=0, cmValL=0, cmValR=0;                                        // Current cm value of Range Sensor (as received from robot)
  uint16_t xl_reading=0, xr_reading=0, yr_reading=0;                            // Reading of joysticks to be sent to robot
  char string_lcd[33] = {0};                                                    // String to be printed on LCD screen

  // Port Initialisation 
  DDRF = 0;               // PORTF into input mode for joysticks
  DDRD &= ~(1<<PD2);      // PORTD into input mode for button
  PORTD |= (1<<PD2);      // Pull up resistor for PD2

  // Setting up interrupt for button
  EICRA |= (1<<ISC21);
	EICRA &= ~(1<<ISC20); // INT2 set falling edge trigger
	EIMSK |= (1<<INT2);   // INT2 enable

  UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

  sei(); // Enable interupts

  while(1) // Main loop
  {
    // Sending information to robot
    // Taken from MXEN2003 Lab 7 Outline
    current_ms = milliseconds_now();

    if (current_ms-last_send_ms >= 100) // Sending rate of 10Hz
    {
      // Read values from joysticks
      yr_reading = adc_read(0);
      xr_reading = adc_read(1);
      xl_reading = adc_read(14);
  
      // Convert the readings from 10-bit to 8-bit
      // Keep reading within upper boundary of 253, to not interfere with start and stop byte
      sendDataByte1 = yr_reading / 4;
      if (sendDataByte1 > 253) {sendDataByte1 = 253;}
      sendDataByte2 = xr_reading / 4;
      if (sendDataByte2 > 253) {sendDataByte2 = 253;}
      sendDataByte3 = xl_reading / 4;
      if (sendDataByte3 > 253) {sendDataByte3 = 253;}

      // sendDataByte4 at min value if autonomy is off
      // or at max if autonomy is on
      if (automode == 0) {sendDataByte4 = 0x00;}
      else {sendDataByte4 = 0xFD;}

      // Set last send time to current time
      last_send_ms = current_ms;

      // Send data to robot as determined above
      // 0xFF is the start byte and OxFE is the end byte, determining the beginning and end of the data to be sent
      serial2_write_byte(0xFF);
      serial2_write_byte(sendDataByte1);
      serial2_write_byte(sendDataByte2);
      serial2_write_byte(sendDataByte3);
      serial2_write_byte(sendDataByte4);
      serial2_write_byte(0xFE);
    }

    if (new_message_received_flag) 
    {
      // If new data has been received, as triggered in the USART2_RX 
      // interrupt convert received data and reset the flag
      cmValL = (dataByte1 * 4);
      cmValF = (dataByte2 * 4);
      cmValR = (dataByte3 * 4);
      new_message_received_flag = false;
    }
    
    // Print the calibrated values of the left and right range sensors 
    // onto the first line of the LCD screen
    lcd_home();
    sprintf(string_lcd, "L: %3u  R: %3u", cmValL, cmValR);
    lcd_puts(string_lcd);

    // Print the state of the autonomous mode and the calibrated value of 
    // the front range sensor onto the second line of the LCD screen
    lcd_goto(0x40);
    if (automode == 0) {sprintf(string_lcd, "AM: OFF F: %3u", cmValF);}
    else {sprintf(string_lcd, "AM: ON  F: %3u", cmValF);}
    lcd_puts(string_lcd);
  }
  return(1);
} //end main 

/***********************************************************************
USART2_RX Interrupt
  Triggered when a new byte is available in the serial buffer
  Reads in incoming information to be used in main function
  Taken from MXEN2003 Lab 7 Outline
Input - None
Output - None
************************************************************************/
ISR(USART2_RX_vect)
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0;   // Data bytes received
	static uint8_t serial_fsm_state=0;									    // Used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2;                          // Move serial byte into variable
	
	switch(serial_fsm_state) // Switch by the current state
	{
		case 0:
      // Do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
      break;

		case 1: // Waiting for first parameter
      recvByte1 = serial_byte_in;
      serial_fsm_state++;
      break;

		case 2: // Waiting for second parameter
      recvByte2 = serial_byte_in;
      serial_fsm_state++;
      break;

		case 3: // Waiting for third parameter
      recvByte3 = serial_byte_in;
      serial_fsm_state++;
      break;

		case 4: // Waiting for stop byte
      if(serial_byte_in == 0xFE) // Stop byte
      {
        // Now that the stop byte has been received, set a flag so that the
        // main loop can execute the results of the message
        dataByte1 = recvByte1;
        dataByte2 = recvByte2;
        dataByte3 = recvByte3;
        new_message_received_flag=true;
      }
      // If the stop byte is not received, there is an error, so no commands are implemented
      serial_fsm_state = 0; // Do nothing next time except check for start byte (below)
      break;
	}
  
  // If start byte is received, go back to expecting the first data byte
	if (serial_byte_in == 0xFF) {serial_fsm_state=1;}
}

/***********************************************************************
INT2 Interrupt
  Triggered the button on the controller is pressed
  Toggles autonomous mode on the robot
Input - None
Output - None
************************************************************************/
ISR(INT2_vect)
{
	uint32_t currTime = milliseconds_now();
	static uint32_t prevTime = 0;
	if ((currTime-prevTime) > DEBOUNCE_PERIOD)
	{
    // If the difference between the current and the previous time the 
    // interrupt was triggered is greater than DEBOUNCE_PERIOD (200ms) then
    // toggle autonomous mode and set the previous time to the current time
		automode ^= 1;
    prevTime = currTime;
	}
}

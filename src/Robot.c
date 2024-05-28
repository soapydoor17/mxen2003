// MXEN2003 Robot Project Code
// Robot.c - code uploaded to the robot system that sends and receives data from the controller system

#include "Robot.h"

volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;  // Data bytes received from the robot
volatile bool new_message_received_flag=false;                        // Triggered when all data has been receieved from the robot

/***********************************************************************
MAIN FUNCTION
  Takes inputs from range sensors, and sends data to controller
  Uses data from controller to either drive manually or autonomously,
  using readings from joysticks or range sensor readings respectively
Input - None
Output - None
************************************************************************/
int main(void)
{
	cli();    // Disable interupts

  // Initialise wireless serial communication and serial monitor
  // as well as libraries for millisecond timer, ADC readings
	serial0_init();
	serial2_init();
  milliseconds_init();
  adc_init();
  _delay_ms(20);
	
	// Variable Initialisation
	// Motor Variables
	uint16_t fc=0, rc=0;                // Joystick readings received from controller
  static int16_t lm=0, rm=0;          // Speed and direction of motors

	// Range sensor and serial communication (sending)
  uint8_t sendDataByte1=0, sendDataByte2=0,sendDataByte3=0;		// Data bytes sent to cnotroller
  uint32_t current_ms=0, last_send_ms=0;			                // Values used for timing the serial send
  uint16_t rsVal_left=2, rsVal_front=0, rsVal_right=0;        // Variables holding ADC readings from range sensors
  uint16_t cmVal_left=0, cmVal_right=0, cmVal_front=0;        // Variables holding calibrated readings from range sensors in centimeters
  uint16_t av_left=0, av_right=0, av_front=0;                 // Variables holding rolling average of range sensor values
  char serial0_sting[100]={0};                                // String to print to serial monitor, containing calibrated range sensor values

  // Left joystick reading for servo control
  uint16_t xl_reading=0;

	// Battery monitor
  uint16_t raw_bat_val=0;   // Variable to store ADC value of batteries' voltage

	// Autonomous
	bool autonomous_flag=0;   // Triggered when button on controller is pressed, putting robot into autonomous mode
	bool turning=false;       // Triggered in autonomous mode to tell robot it should do a hard turn
	bool leftRight=false;     // Tells robot which way to turn when turning is triggered. False for left, true for right

  // Port Initialising
  DDRA |= 0xFF;       // PORTA into output mode for motors & battery detector
  PORTA = 0;				  // PORTA pins originally all off.
	DDRF = 0;           // PORTF into input mode for range sensor

	// PWM set up for servo
	uint16_t compValServo = 1000;       // Variable to which duty cycle will be assigned to
  DDRB |= (1<<PB5);                   // Set PB5 to output mode for motors
  OCR1A = compValServo;               // Initialise comparison values for servo
  TCCR1A |= (1<<COM1A1);              // OC1A clears on compare match
  TCCR1B |= (1<<WGM13)|(1<<CS11);     // WGM 8 and prescaler of 8
  TCNT1 = 0;                          // Set timer/counter at 0
  ICR1 = 20000;                       // TOP is 20000

	// PWM Set up for Motors
	DDRE |= (1<<PE3)|(1<<PE4);              // PE3 and PE4 to output mode for motors
	OCR3A = 8000;                           // Set initial speed of left and right motor at 8000
	OCR3B = 8000;
	TCCR3A |= (1<<COM3A1) | (1<<COM3B1);    // OC3A and OC3B clears on compare match
	TCCR3B |= (1<<WGM33) | (1<<CS31);       // WGM 8 and Prescaler of 8
	ICR3 = 10000;                           // TOP is 10000

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

	sei();    // Enable interupts

	while(1)  // Main loop
	{
    // Sending information to robot
    // Taken from MXEN2003 Lab 7 Outline
    current_ms = milliseconds_now();

		if (current_ms-last_send_ms >= 100) // Sending rate of 10Hz
		{
      // Left sensor
      // Read the sensor value and add it to the rolling average
      // Convert to centimenter value accoding to calibration - SEE REPORT!!
      // Convert the readings from 10-bit to 8-bit
      // Keep reading within upper boundary of 253, to not interfere with start and stop byte
			rsVal_left = adc_read(0);
			av_left = 0.7 * av_left + 0.3 * rsVal_left;
			cmVal_left = (2516/av_left);
			sendDataByte1 = cmVal_left/4;
			if (sendDataByte1>253) {sendDataByte1 = 253;}
	
      // Front sensor - same process as left sensor
			rsVal_front = adc_read(1);
			av_front = 0.7 * av_front + 0.3 * rsVal_front;
			cmVal_front = (2516/av_front);
			sendDataByte2 = cmVal_front /4;
			if (sendDataByte2>253) {sendDataByte2 = 253;}
	
      // Right sensor - same process as left sensor
			rsVal_right = adc_read(2);
			av_right = 0.7 * av_right + 0.3 * rsVal_right;
			cmVal_right = (2516/av_right);
			sendDataByte3 = cmVal_right / 4;
			if (sendDataByte3>253) {sendDataByte3 = 253;}
	
      // Send data to robot as determined above
      // 0xFF is the start byte and OxFE is the end byte, determining the beginning and end of the data to be sent
			last_send_ms = current_ms;
			serial2_write_byte(0xFF);
			serial2_write_byte(sendDataByte1);
			serial2_write_byte(sendDataByte2);
			serial2_write_byte(sendDataByte3);
			serial2_write_byte(0xFE);
    }
		
		if (new_message_received_flag)
		{
      // If new data has been received, as triggered in the USART2_RX 
      // interrupt convert received data

			// Servo - Controlled by left joystick on the controller
      // Times by four to convert from 8-bit to 10-bit
      // Add 1000 so compValServo is in between 1000 tp 2024
			xl_reading = dataByte3 * 4;
			compValServo = xl_reading + 1000;
			OCR1A = compValServo;

			// Motor Speeds - Controlled by right joystick on the controller
      // Forward component of joystick reading (fc) makes both left and right motor (lm and rm) go forward
      // Right component of joystick reading (rc) increases left motor and decreases right motor
      // Robot should be stationary when joystick is centered, hence fc and rc should be offset by 253/2
			fc=dataByte1, rc=dataByte2;
			rm = fc + rc - 253;
			lm = fc - rc;

      // Autonomous Mode
			if (dataByte4 == 0x00) {autonomous_flag = 0;}
			else {autonomous_flag = 1;}
      
			if (autonomous_flag == 0)
			{
        // MANUAL CONTROL

        // Set speed of motors according to absolute value of the above calculations
				OCR3A = (int32_t)abs(lm) * 10000 / 126;
				OCR3B = (int32_t)abs(rm) * 10000 / 126;

				if (lm>=0)
				{
					// Left motor goes forward if the calculated speed is positive 
					PORTA |= (1<<PA0);
					PORTA &= ~(1<<PA1);
				}
				else
				{
          // Left motor goes backward if the calculated speed is negative
          PORTA &= ~(1<<PA0);
          PORTA |= (1<<PA1);
				}

				if (rm>=0)
				{
					// Right motor goes forward if the calculated speed is positive 
					PORTA |= (1<<PA2);
					PORTA &= ~(1<<PA3);
				}
				else
				{
					// Right motor goes backward if the calculated speed is negative
					PORTA &= ~(1<<PA2);
					PORTA |= (1<<PA3);
				}
			}
      
			else
			{	
        // AUTONOMOUS FUNCTION
				if (turning == false)
				{
					// WALL FOLLOWING
					if(cmVal_front > 8)
					{
						//move forward
						PORTA |= (1<<PA1);
						PORTA &= ~(1<<PA0);
						PORTA |= (1<<PA3);
						PORTA &= ~(1<<PA2);

						//adjust position
						if (cmVal_right <= 9)
						{
							// Veer left
							OCR3B = 6000;
							OCR3A = 2000;
						}
						if (cmVal_left <= 9)
						{
              // Veer right
							OCR3A = 6000;
							OCR3B = 2000;
						}
						if (cmVal_left > 9 && cmVal_right > 9)
						{
              // Go straight forward
							OCR3A = 4000;
							OCR3B = 4000;
						}

					}

          // Too close to wall and turn
					if (cmVal_front <= 8)
					{
						// Stop
						PORTA &= ~(1<<PA0);
						PORTA &= ~(1<<PA1);
						PORTA &= ~(1<<PA2);
						PORTA &= ~(1<<PA3);

						// Enable turning and decide if turning left or right
						turning = true;
						if (cmVal_left > cmVal_right) {leftRight = false;}
						else {leftRight = true;}
					}
				}

				else
				{
					// TURNING 
          // get out of turning mode if front sensor can see more than 30cm
					if (cmVal_front > 30) {turning = false;}

					else if (leftRight == false)
					{
						// Turn left if left turn is true
						OCR3A = 3500;
						OCR3B = 3500;
						PORTA |= (1<<PA0);
						PORTA &= ~(1<<PA1);
						PORTA |= (1<<PA3);
						PORTA &= ~(1<<PA2);
					}

					else
          {
						// Else turn right
						OCR3A = 3500;
						OCR3B = 3500;
						PORTA |= (1<<PA1);
						PORTA &= ~(1<<PA0);
						PORTA |= (1<<PA2);
						PORTA &= ~(1<<PA3);
					}
				}
			}
      new_message_received_flag = false;    // Reset flag
		}

  // Print the calibrated range sensor values onto the serial monitor
  sprintf(serial0_sting, "LEFT: %5ucm     FRONT: %5ucm     RIGHT: %5ucm \n", cmVal_left, cmVal_front, cmVal_right);
	serial0_print_string(serial0_sting);
  
	// Battery Monitor
  // If the adc reading from the batteries is less than or equal to 7V (995 bits)
  // then turn on the LED (connected to PA5)
  raw_bat_val = adc_read(3);
  if (raw_bat_val <= 995) {PORTA |= (1<<PA5);}
  else {PORTA &= ~(1<<PA5);}
	}
	return(1);
} // End main 

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
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;    // Data bytes received
	static uint8_t serial_fsm_state=0;									                  // Used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2;                                        // Move serial byte into variable
	
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

		case 4: // Waiting for fourth parameter
      recvByte4 = serial_byte_in;
      serial_fsm_state++;
      break;

    case 5: // Waiting for stop byte
      if(serial_byte_in == 0xFE) // Stop byte
      {
        // Now that the stop byte has been received, set a flag so that the
        // main loop can execute the results of the message
        dataByte1 = recvByte1;
        dataByte2 = recvByte2;
        dataByte3 = recvByte3;
        dataByte4 = recvByte4;
        new_message_received_flag=true;
      }
      // If the stop byte is not received, there is an error, so no commands are implemented
      serial_fsm_state = 0; // Do nothing next time except check for start byte (below)
      break;
	}

  // If start byte is received, we go back to expecting the first data byte
	if (serial_byte_in == 0xFF) {serial_fsm_state=1;}
}

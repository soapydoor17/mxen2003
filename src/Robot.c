//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4 = 0;
volatile bool new_message_received_flag=true; // NOTE: Make false

int main(void)
{
	cli();
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
  	milliseconds_init();
  	adc_init();
  	_delay_ms(20);
	
	// variables
	//motors
	uint16_t fc=0, rc=0;              // Joystick readings received from controller
  	static int16_t lm=0, rm =0;       // Speed and direction of motors
   	char serial0_sting[100] = {0};
	//autonomous
	bool autonomous_flag= 1;

	//range sensor and serial communication (sending)
  	uint8_t sendDataByte1=0, sendDataByte2=0,sendDataByte3=0;		// data bytes sent
  	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
  	uint16_t rsVal_left = 2, rsVal_front = 0, rsVal_right = 0, cmVal_left = 0, cmVal_right = 0, cmVal_front = 0, av_left=0, av_right=0, av_front=0;       // range sensor value
  	uint16_t xl_reading=0;
	bool turning = false;
	bool leftRight = false; // false for left, true for right

	//battery checker
  	uint16_t raw_bat_val = 0; // value for adc read of battery to be stored

  	//port initialising
  	DDRA |= 255;      // put PORTA into output mode for motors & battery detector
  	PORTA = 0;						// PORTA pins originally all off.

	// PWM set up for servo
	uint16_t compValServo = 1000;
  	DDRB |= (1<<PB5);
  	OCR1A = compValServo;
  	TCCR1A |= (1<<COM1A1);
  	TCCR1B |= (1<<WGM13)|(1<<CS11);
  	TCNT1 = 0;
  	ICR1 = 20000;

	// PWM Set up for Motors
	DDRE |= (1<<PE3)|(1<<PE4);        // PORTE output mode for motors
	OCR3A = 8000;
	OCR3B = 8000;
	TCCR3A |= (1<<COM3A1) | (1<<COM3B1);    // OC3A and OC3B clears on compare match
	TCCR3B |= (1<<WGM33) | (1<<CS31);       // WGM 8 and Prescaler of 8
	ICR3 = 10000;                           // TOP is 16000

	//UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	DDRF = 0;  //input for range sensor
	sei();

	while(1)//main loop
	{
		rsVal_left = adc_read(0); // Left sensor
		av_left = 0.7 * av_left + 0.3 * rsVal_left;
		cmVal_left = (2516/av_left);
		sendDataByte1 = cmVal_left/4;

		rsVal_front = adc_read(1); // Front sensor
		av_front = 0.7 * av_front + 0.3 * rsVal_front;
		cmVal_front = (2516/av_front);

		rsVal_right = adc_read(2); // Right sensor
		av_right = 0.7 * av_right + 0.3 * rsVal_right;
		cmVal_right = (2516/av_right);

		//sending range sensor data value section
		/*
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{
			rsVal_left = adc_read(0); // Left sensor
			av_left = 0.7 * av_left + 0.3 * rsVal_left;
			cmVal_left = (2516/av_left);
			sendDataByte1 = cmVal_left/4;
			if(sendDataByte1>253)
			{sendDataByte1 = 253;}
	
			rsVal_front = adc_read(1); // Front sensor
			av_front = 0.7 * av_front + 0.3 * rsVal_front;
			cmVal_front = (2516/av_front);
			sendDataByte2 = cmVal_front /4;
			if(sendDataByte2>253)
			{sendDataByte2 = 253;}
	
			rsVal_right = adc_read(2); // Right sensor
			av_right = 0.7 * av_right + 0.3 * rsVal_right;
			cmVal_right = (2516/av_right);
			sendDataByte3 = cmVal_right / 4;
			if(sendDataByte3>253)
			{sendDataByte3 = 253;}
	
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2);
			serial2_write_byte(sendDataByte3);
			serial2_write_byte(0xFE); 		//send stop byte = 254
    	}
		*/
		
		//motor control
		if (new_message_received_flag)
		{
			// Servo
			xl_reading = dataByte3 * 4;
			compValServo = xl_reading + 1000;
			OCR1A = compValServo;

			// Motor
			fc=dataByte1, rc=dataByte2;

			rm = fc + rc - 253;
			lm = fc - rc;

			if (dataByte4 == 0x00) {autonomous_flag = 0;}
			else {autonomous_flag = 1;}
			autonomous_flag = 1; // NOTE: Delete
			if(autonomous_flag == 0)
			{
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
			}
			//autonomous funciton
			else
			{	
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
							OCR3A = 6000;
							OCR3B = 2000;
						}
						if (cmVal_left > 9 && cmVal_right > 9)
						{
							OCR3A = 4000;
							OCR3B = 4000;
						}

					}
					if(cmVal_front <= 8)
					{
						//stop
						PORTA &= ~(1<<PA0);
						PORTA &= ~(1<<PA1);
						PORTA &= ~(1<<PA2);
						PORTA &= ~(1<<PA3);

						// turn
						turning = true;
						if (cmVal_left > cmVal_right) {leftRight = false;}
						else {leftRight = true;}
					}
				}
				else
				{
					// TURNING 
					if (cmVal_front > 50)
					{turning = false;}
					else if (leftRight == false)
					{
						// turn left
						OCR3A = 3500;
						OCR3B = 3500;
						PORTA |= (1<<PA0);
						PORTA &= ~(1<<PA1);
						PORTA |= (1<<PA3);
						PORTA &= ~(1<<PA2);
					}
					else{
						// turn right
						OCR3A = 3500;
						OCR3B = 3500;
						PORTA |= (1<<PA1);
						PORTA &= ~(1<<PA0);
						PORTA |= (1<<PA2);
						PORTA &= ~(1<<PA3);
					}
				}
			}
		}
	// NOTE: new_message_received_flag = false;
  		
	
	sprintf(serial0_sting, "LEFT: %5ucm     FRONT: %5ucm     RIGHT: %5ucm \n", cmVal_left, cmVal_front, cmVal_right);
	serial0_print_string(serial0_sting);  // print the received bytes to the USB serial to make sure the right messages are received
		
	//battery checking
  	raw_bat_val = adc_read(3);
  	if (raw_bat_val <= 995)
  	{
  		PORTA |= (1<<PA5);	
  	}
  	else
  	{
  		PORTA &= ~(1<<PA5);
  	}
	}
	return(1);
}//end main 

ISR(USART2_RX_vect)
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
		case 3: //waiting for third parameter
    	recvByte3 = serial_byte_in;
    	serial_fsm_state++;
    	break;
		case 4: //waiting for fourth parameter
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

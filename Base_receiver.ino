/*
Name:		Base_receiver.ino
Created:	1/13/2016 8:58:21 AM
Author:	Eason Smith (Eason@EasonRobotics.com)

| NES Receiver Software |
Developed to receive controller button data via a hopeRF receiver module and transfer that data via USB to a PC, 
appearings as a regular USB HID Joystick in windows. 

NOTE FROM THE AUTHOR - 1/25/16 - This project is one of my first electronics projects I ever did (circa 2010) and is no
longer in active development. While the code probably still compiles for the Arduino IDE version 0022 or so, I have made small 
code changes and comment cleanups to it's more cringe-worthy parts for presentation purposes over the years and it's current 
compilation status is unknown. Also because I was developing both NES and SNES versions of this code at the time, there may be 
artifacts left over from that code. I have either commented that stuff out, or streamlined the code to work with the NES version
as well.

Requires:
Arduino UNO (Hardware)
Virtual Wire library - which can be found here: http://www.airspayce.com/mikem/arduino/VirtualWire/index.html

See VirtualWire.h for detailed API docs
Author: Mike McCauley (mikem@open.com.au)
Copyright (C) 2008 Mike McCauley

The joyreport struct is based on the work of Darren Hunt. More on his work can be found here: http://hunt.net.nz/users/darran/

PINS
0 - 
11 --->HopeRF RX
*/

#include <VirtualWire.h>

#define DEBUG
#define PRESSED 0

byte message[3];             // 3 byte message

typedef struct t_joyReport 
{
	int8_t x;
	int8_t y;
	uint32_t buttons;
} t_joyReport;

t_joyReport joyReport;

/////////////////////////////////SETUP/////////////////////////////////////////////////////////

void setup()
{
	delay(4000);

// DEBUG STUFF  
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTBIN(x)     Serial.print (x, BIN)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define DEBUG_PRINTLNBIN(x)  Serial.println (x, BIN)
#define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
#define DEBUG_PRINTLNDEC(x)  Serial.println (x, DEC)
DEBUG_PRINTLN("DEBUG initialized");

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTBIN(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNBIN(x)
#define DEBUG_PRINTDEC(x) 
#define DEBUG_PRINTLNDEC(x)
#endif
	
	Serial.begin(115200);
	DEBUG_PRINTLN("8U2 @ 115200");

	//setup virtualwire
	vw_setup(4800);	
	vw_rx_start();                               
	
	DEBUG_PRINTLN("Vwire @ 4800bps");
	DEBUG_PRINTLN("Setup Complete. Starting Program.");

}

///////////MAIN LOOP///////////
void loop() 
{
	//receive message info
	byte buf[VW_MAX_MESSAGE_LEN];               
	uint8_t buflen = VW_MAX_MESSAGE_LEN; 

	// if message available
	if (vw_get_message(buf, &buflen))
	{
		//put received bytes into a seperate array
		for (int t = 0; t < buflen; t++) {
			message[t] = buf[t];
		}

		//get controller id type (NES, Atari, etc.)
		byte cID = message[0];

		//assemble for report
		if (cID = 0x01) { //NES
			
			byte right = (message[1] >> 0) & 1; 
			byte left = (message[1] >> 1) & 1;
			byte down = (message[1] >> 2) & 1;
			byte up = (message[1] >> 3) & 1;
			
			joyReport.y = 0;
			joyReport.x = 0;

			//if up
			if (up == PRESSED) {
				joyReport.y = 127;
			}

			//if down
			else if (down == PRESSED) {
				joyReport.y = -127;
			}
			
			//if right
			if (right == PRESSED) {
				joyReport.x = 127;
			}

			//if left
			else if (left == PRESSED) {
				joyReport.x = -127;
			}
		}

		//assemble remaining buttons
		joyReport.buttons = ((uint32_t)message[2]) << 4;
		joyReport.buttons |= message[1] >> 4;

		//send report via USB
		sendJoyReport(&joyReport);
		
#ifdef DEBUG
		debug();
#endif  
	}

	else
	{
		DEBUG_PRINTLN("no message available");

		//and send previous joyreport
		sendJoyReport(&joyReport);
	}
}


/////////////////////////////////////////END MAIN LOOP//////////////////////////////////////////////////

//send joystick report to the ATmega8U2 chip 
void sendJoyReport(struct t_joyReport *report)
{
	Serial.write( (uint8_t*)report , sizeof(t_joyReport) );
}

//show messages received
void debug()
{
	DEBUG_PRINTLN("RX:");
	// addr
	DEBUG_PRINTLNBIN(message[0]);
	//  buttons
	DEBUG_PRINTLNBIN(message[1]);
	DEBUG_PRINTLNBIN(message[2]);

}

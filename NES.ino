/*
Name:		Controller.ino
Created:	1/13/2016 8:58:21 AM
Author:	Eason Smith (Eason@EasonRobotics.com)

| NES Transmitter Software |
Developed to read input buttons from an NES controller and send them wirelessly using a HopeRF transmitter to an Arduino Uno
base unit with a paired HopeRF receiver.

NOTE FROM THE AUTHOR - 1/24/16 - This project is one of my first electronics projects I ever did (circa 2010) and is no
longer in active development. While the code should still compile for the Arduino IDE version 0022, I have made small code and 
comment cleanups over the years to it's more cringe-worthy parts in order to make it more presentable. It still suffers from 
some common (novice-programmer) issues. Obvious issues are: too many globals, too many comments, not OOP, 
lack of pass-by-reference parameters and no return variables, etc. I will fix it all eventually. If you find mistakes, feel free 
to email me or submit a pull request. Also please mind the commented artifacts from Atari and SNES controller versions, 
which you might find interesting if you'd like to know what modifications would be required to read those controllers. 
A wireless Atari controller was originally developed (as a stepping stone) with a very primative version of this code, and so
parts are still remaining for later use. (Atari controller protocal was widely adopted by other systems at the time)

Features:
-Power on, Power off, Sleep mode, Low battery warning and LED effects.
-Power down/Sleep mode on button inactivity.
-4800bps data rate
-Wake from sleep on button press.
-Active battery voltage level monitoring
-NES data directly read from the NES controller's shift register IC (Clock, Latch and Data wires)

Requires:
Virtual Wire library - which can be found here: http://www.airspayce.com/mikem/arduino/VirtualWire/index.html
AVR/Sleep library (comes with the arduino IDE)
MemoryFree library (optional. Still looking around for this one)

PINS
0 - Rx - used for waking and is physically connected to an NES button pin on the shift register
1 - 
2 - Wakepin input connected to a resistor which is connected to pin0 and NES wake button.
3 - NES Latch pin/wire
4 - NES Clock pin/wire
5 - NES Data pin/wire
6
7
8
9 - Left NES logo Led
10 - Right NES logo Led
11 
12 - tx -->RF Link transmitter
A5/19 - Battery read line
*/

#include <VirtualWire_Config.h>
#include <avr/sleep.h>
#include <VirtualWire.h>

//debug stuff
#define DEBUG
#define DEBUGBUTTONS
#define DEBUGLATENCY
//#define DEBUGBATTERY
//#define DEBUGMEMORY
#ifdef DEBUGMEMORY
#include <MemoryFree.h>
#endif

//NES Controller stuff
const byte NES_PIN_LATCH = 3;
const byte NES_PIN_CLOCK = 4;
const byte NES_PIN_DATA = 7;
byte controller_data = 0;			// temporary nes controller data container
byte mybinary = 0xff;					
byte mybinary2 = 0xff;
byte buttons[8] = { 1,1,1,1,1,1,1,1 };
byte buttons2[8] = { 1,1,1,1,1,1,1,1 };

/*
//Atari Controller Related
byte inputs[] = {2,3,4,5,6,7,8,18};           // pin 18 is not used
byte inputs2[] = {16,15,14,11,17,10,9,17};    // same with pin 17
#define NUMBUTTONS sizeof(inputs)
#define NUMBUTTONS2 sizeof(inputs2)
*/

//LED stuff
const byte LED_LEFT_PIN = 5; //D5
const byte LED_RIGHT_PIN = 6;

//battery stuff
const byte BATTERY_PIN = 5;//Analog pin 5 =battery voltage
const byte numReadings = 10;    //num battery readings you want to average.
int readings[numReadings];   
int battery_total = 0;		
int battery_avg_voltage = 0;    

//long howLongToWait = 1000*60*5;     // check battery every 5 minutes
unsigned long lastTimeBatteryChecked = 0;       
unsigned long howLongSinceBatteryChecked;      

#ifdef DEBUGBATTERY
float floatAvg = 0;    // float used for debug
#endif

//Latency related
#ifdef DEBUGLATENCY
byte latency;
long currentTime = 0;
long prevlatencycheck = 0;
#endif

//sleep related
const byte WAKE_PIN = 2;           
long lastTimePressed = 0;
const long sleepinterval = 1000L * 60L * 7L;     // wait time till sleep. default is 7L == 7 minutes. use 15000L for debug

#ifdef DEBUGBUTTONS
unsigned char test[5]; //debug test variable as large as the number of buttons
#endif

//flags
bool startupflag = true;
bool lowbatteryflag = false;
bool blinkAwakeFlag = false;

//buttons
const byte NUMBUTTONS = 8; 

void setup() 
{
	//begin serial for debug
	Serial.begin(115200);

	// begin VirtualWire @4800bps
	vw_setup(4800);     

	//define what certain debug stuff means if debug is turned on
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTBIN(x)     Serial.print (x, BIN)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define DEBUG_PRINTLNBIN(x)  Serial.println (x, BIN)
#define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
#define DEBUG_PRINTLNDEC(x)  Serial.println (x, DEC)
#define DEBUG_PRINTFLT(x)  Serial.print (x, 2)
#define DEBUG_PRINTLNFLT(x)  Serial.println (x, 2)
DEBUG_PRINTLN("debug initialized");

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTBIN(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNBIN(x)
#define DEBUG_PRINTDEC(x) 
#define DEBUG_PRINTLNDEC(x)
#define DEBUG_PRINTFLT(x)
#define DEBUG_PRINTLNFLT(x)
#endif

	//set NES inputs and outputs. Set wake pin as input. set LED pins as output
	//DDRD |= (1 << LED_LEFT_PIN | 1 << LED_RIGHT_PIN | 1 << NES_PIN_LATCH | 1 << NES_PIN_CLOCK);
	//DDRD &= ~(1 << NES_PIN_DATA | 1 << WAKE_PIN);

	// start latch and clock pins high. enable wakepin pullup resistor
	//PORTD |= (1 << WAKE_PIN | 1 << NES_PIN_LATCH | 1 << NES_PIN_CLOCK);

	//old way
	pinMode(LED_LEFT_PIN, OUTPUT);    
	pinMode(LED_RIGHT_PIN, OUTPUT);  
	pinMode(NES_PIN_LATCH, OUTPUT);   
	pinMode(NES_PIN_CLOCK, OUTPUT);   
	pinMode(NES_PIN_DATA, INPUT);    
	pinMode(WAKE_PIN, INPUT); 
	
	digitalWrite(WAKE_PIN, HIGH);   
	digitalWrite(NES_PIN_LATCH, HIGH);
	digitalWrite(NES_PIN_CLOCK, HIGH);
	

	//populate 10 quick voltage readings into the battery array on startup
	for (byte thisReading = 0; thisReading < numReadings; thisReading++)
	{
		readings[thisReading] = analogRead(BATTERY_PIN);
		battery_total += readings[thisReading];
		battery_avg_voltage = battery_total / numReadings;
		
		#ifdef DEBUGBATTERY
		floatAvg = battery_avg_voltage * .0048; //ADC conversion
		#endif
	}

	// [ATARI CONTROLLERS ONLY] Make all of the pins we are using set to input & enable pull-up resistors on switch pins so that we can read when they are pressed or not
	///*
	//for (int i=0; i < NUMBUTTONS; i++){                            //do this "numbutton" times.
	//    pinMode(inputs[i], INPUT);                                 //enables the pin as an input
	//    digitalWrite(inputs[i], HIGH);// enables pull up
	//      }
	// for (int i=0; i < NUMBUTTONS2; i++){                           //do this "numbutton" times.
	//    pinMode(inputs2[i], INPUT);                                //enables the pin as an input
	//    digitalWrite(inputs2[i], HIGH);// enables pull up
	//      }
	//*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//pins are now defined and set, virtual wire is ready to go and serial debug has been turned on.
	DEBUG_PRINTLN("begin program");

	//insert startup LED sequence here
	startupLED();

}


void loop() 
{
	//check for wakeup condition
	if (blinkAwakeFlag) {
		DEBUG_PRINTLN("blinking awake");
		fastBlink();
		blinkAwakeFlag = false;
	}

	readBattery();

	//Check battery reading every 1 minute
	long howLongToWait = 10000 * 60;    
	howLongSinceBatteryChecked = millis() - lastTimeBatteryChecked;
	if (howLongSinceBatteryChecked >= howLongToWait || startupflag == true)
	{
		if (battery_avg_voltage < 650) // < 2.9 volts
		{
			// sleep
			DEBUG_PRINTLN("battery criticly low. Shutting down");
			emergencyShutdown();
		}

		else if (battery_avg_voltage < 700) // < 3.3 Volts
		{
			DEBUG_PRINTLN("battery low");
			lowbatteryflag = true;	
		}

		else if (battery_avg_voltage > 700) // > 3.3 Volts
		{
			DEBUG_PRINTLN("battery ok");
			bothLedsON();	
		}

		lastTimeBatteryChecked = millis();
		startupflag = false;		                                  
	}

	//If battery is low, flash the low battery LED 
	if (lowbatteryflag == true) 
	{
		lowBatteryWarning();
	}
	
	//read Atari buttons
	//check_buttons(); 
	//check_buttons2();  

	controllerRead();
	order_buttons();
	bitshift(mybinary);                //buttons 0 through 7

	//if SNES
	if (NUMBUTTONS > 8)
	{
		bitshift(mybinary2);          //buttons 8 through 15 
	}
	
	//prepare the message
	char msg[3];//
	msg[0] = 0x01; // controller ID 0x01 (means NES)
	msg[1] = mybinary; //button set 1

	if (NUMBUTTONS > 8) 
	{ 
		//SNES - include more than 8 buttons
		msg[2] = mybinary2; 
	}

#ifdef DEBUGBUTTONS
	for (byte u = 0; u < 3; u++) 
	{  
		test[u] = msg[u];
	}
	DEBUG_PRINTLNBIN(test[0]);
	DEBUG_PRINTBIN(test[1]);
	DEBUG_PRINTLNBIN(test[2]);
#endif

	//send the message
	vw_send((uint8_t *)msg, strlen(msg));

	// Wait until the whole message is gone
	vw_wait_tx();
	
#ifdef DEBUGLATENCY
	currentTime = millis();
	latency = (currentTime - prevlatencycheck);
	DEBUG_PRINT("L:");
	DEBUG_PRINTDEC(latency);
	prevlatencycheck = currentTime;
#endif

#ifdef DEBUGBATTERY
	DEBUG_PRINT(" MS, B:");
	//DEBUG_PRINTDEC(battery_avg_voltage);   //(for if you'd rather display an int battery_avg_voltage. Adds a little latency, harder to read.)
	DEBUG_PRINTFLT(floatAvg);   //(for if you'd rather display a float average. Adds more latency, easier to read.)
#endif 

#ifdef DEBUGMEMORY
								//DEBUG_PRINTLN();    //end on a new line
	DEBUG_PRINT(" M:");
	DEBUG_PRINT(freeMemory());
#endif

#ifdef DEBUG
	DEBUG_PRINTLN("");
#endif

	////if its been 7 minutes without any button presses go to sleep
	unsigned long now = millis();
	if (now - lastTimePressed > sleepinterval) 
	{ 
		// use interrupt 0 (pin 2) and callback wakeupnow
		attachInterrupt(0, wakeUpNow, LOW); 

		//notify
		DEBUG_PRINTLN("Entering Sleep mode");
		DEBUG_PRINT("time elapsed since last press: ");
		DEBUG_PRINTLN((now - lastTimePressed) / 1000);
		DEBUG_PRINT("Seconds");

		// this is now the last time i pressed something
		lastTimePressed = now;

		//LED warning
		slowBlink();

		//nessesary to prevent serial issues
		delay(100);

		//go to sleep
		sleepNow();
	}

//end of main loop
}

/////////////FUNCTIONS//////////////////////////////

//sleepNow puts the controller to sleep
void sleepNow()
{
	/* In the Atmega8 datasheet http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
	* there is a list of sleep modes which explains which clocks and
	* wake up sources are available in which sleep mode.
	*
	* In the avr/sleep.h file, the call names of these sleep modes are to be found:
	*
	* The 5 different modes are:
	*     SLEEP_MODE_IDLE         -the least power savings
	*     SLEEP_MODE_ADC
	*     SLEEP_MODE_PWR_SAVE
	*     SLEEP_MODE_STANDBY
	*     SLEEP_MODE_PWR_DOWN     -the most power savings
	*/
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   

	sleep_enable();                // enables the sleep bit in the mcucr register so sleep is possible. 

								   /* Now it is time to enable an interrupt. We do it here so an
								   * accidentally pushed interrupt button doesn't interrupt
								   * our running program. if you want to be able to run
								   * interrupt code besides the sleep function, place it in
								   * setup() for example.
								   *
								   * In the function call attachInterrupt(A, B, C)
								   * A   can be either 0 or 1 for interrupts on pin 2 or 3.
								   *
								   * B   Name of a function you want to execute at interrupt for A.
								   *
								   * C   Trigger mode of the interrupt pin. can be:
								   *             LOW        a low level triggers
								   *             CHANGE     a change in level triggers
								   *             RISING     a rising edge of a level triggers
								   *             FALLING    a falling edge of a level triggers
								   *
								   * In all but the IDLE sleep modes only LOW can be used.
								   */

	attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run wakeupnow() when pin 2 gets LOW (button is pressed)

	DEBUG_PRINTLN("Zzz...");        

	sleep_mode();            // sleeps here


	// THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP//

	sleep_disable();         
							 
	detachInterrupt(0);      // disables interrupt 0 on pin 2 
							 
	DEBUG_PRINTLN("...waking up.");

}


///Wake procedure. Fast blink if we wake.
void wakeUpNow()
{
	blinkAwakeFlag = true;    // fast blink upon waking.
}


//quickly blink 7 times.
void fastBlink() 
{
	for (byte b = 0; b < 7; b++) 
	{
		bothLedsOFF();
		delay(100);
		bothLedsON();
		delay(100);
	}
}

//slowly blink 3 times.
void slowBlink() 
{
	for (byte b = 0; b < 3; b++) 
	{
		bothLedsON();
		delay(1000);
		bothLedsOFF();
		delay(1000);
	}
}

///Read data from the NES controller's on board shift register.
void controllerRead() 
{
	controller_data = 0;

	digitalWrite(NES_PIN_LATCH, LOW);
	digitalWrite(NES_PIN_CLOCK, LOW);
	//PORTD &= ~(1 << NES_PIN_LATCH | 1 << NES_PIN_CLOCK); 
	
	digitalWrite(NES_PIN_LATCH, HIGH);
	delayMicroseconds(2);
	digitalWrite(NES_PIN_LATCH, LOW);

	//open the data line
	controller_data = digitalRead(NES_PIN_DATA);

	//collect the state of every button
	for (byte i = 0; i < NUMBUTTONS; i++) 
	{
		digitalWrite(NES_PIN_CLOCK, HIGH);

		delayMicroseconds(2);

		controller_data <<= 1; //shift 1 to the left
		controller_data |= digitalRead(NES_PIN_DATA); //add button
		
		delayMicroseconds(4);
		
		digitalWrite(NES_PIN_CLOCK, LOW);
	}
	
	//if a button was pressed, reset sleep timer / prevent sleep
	if (controller_data != 255) 
	{
		lastTimePressed = millis();
	}  //remember
}

///Organize buttons into the order expected by the reciever
void order_buttons() 
{
	//check button, throw it into a temporary variabe
	buttons[3] = bitRead(controller_data, 0); // ---right button (LSB) B00000001
	buttons[2] = bitRead(controller_data, 1); // ---left button
	buttons[1] = bitRead(controller_data, 2); // ---down button
	buttons[4] = bitRead(controller_data, 3); // ---up button
	buttons[7] = bitRead(controller_data, 4); // ---Start button
	buttons[6] = bitRead(controller_data, 5); // ---select button
	buttons[0] = bitRead(controller_data, 6); // ---B button
	buttons[5] = bitRead(controller_data, 7); // -- A button (MSB) B10000000
	
	//buttons2[0]= bitRead(controller_data, 8); // ------order buttons for SNES.
}


///Bitshift Procedures//

//lower numbered pins on the right, higher pins on the left.
void bitshift(byte &buttonbyte) 
{
	buttonbyte = buttons[0] | (buttons[1] << 1) | (buttons[2] << 2) | (buttons[3] << 3) | (buttons[4] << 4) | (buttons[5] << 5) | (buttons[6] << 6) | (buttons[7] << 7);
}

// reads the battery level, computes a running average of last
void readBattery() 
{
	//remove stale reading
	static byte battery_index = 0;
	battery_total -= readings[battery_index];

	// get new battery voltage reading
	readings[battery_index] = analogRead(BATTERY_PIN);

	// add to total. advance circular array.
	battery_total += readings[battery_index++];

	if (battery_index >= numReadings)
	{
		battery_index = 0;
	}
	
	// save average voltage
	battery_avg_voltage = battery_total / numReadings;

	#ifdef DEBUGBATTERY
	floatAvg = battery_avg_voltage * .0048;
	#endif
}

//intial slow blink routine when the controller is turned on
void startupLED() 
{
	// fade in Left LED in increments of 10
	for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) 
	{
		analogWrite(LED_LEFT_PIN, fadeValue);
		
		delay(10);
	}

	// fade out left LED and fade in right LED
	for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 10) 
	{
		analogWrite(LED_LEFT_PIN, fadeValue);  
		analogWrite(LED_RIGHT_PIN, 255 - fadeValue);    
		
        delay(10); 
	}

	//fade out right LED
	for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) 
	{
		analogWrite(LED_RIGHT_PIN, 255 - fadeValue);  //fade
													      
		delay(10);
	}

	//fade in both LED's simultaneously
	for (int fadeValue = 0; fadeValue <= 255; fadeValue += 10) 
	{
		analogWrite(LED_LEFT_PIN, fadeValue);       
		analogWrite(LED_RIGHT_PIN, fadeValue);    
												      
		delay(10);
	}

	//fast blink to signal controller coming online
	fastBlink();
}


// lowBatteryWarning - notifys the user via pulsing leds when the battery is low. uses cos function
void lowBatteryWarning() 
{
	int period = 1000; //ms
	int displacement = 1000;
	unsigned long cur_time = millis();

	byte leftvalue = 128 + 127 * cos(TWO_PI / period*cur_time);
	byte rightvalue = 128 + 127 * cos(TWO_PI / period*(displacement - cur_time));
	
	analogWrite(LED_LEFT_PIN, leftvalue);
	analogWrite(LED_RIGHT_PIN, rightvalue);
}

//emergencyShutdown - performs shutdown procedure and starts LED notification
void emergencyShutdown()
{
	DEBUG_PRINTLN("Emergency! Voltage is at:");
	DEBUG_PRINTDEC(battery_avg_voltage);
	DEBUG_PRINTLN("Shutding down!");

	// LED's flash SOS twice
	SOS();
	SOS();

	sleep_mode();
}


// turn LEDS on
void bothLedsON() 
{
	PORTD |= ((1 << LED_LEFT_PIN) | (1 << LED_RIGHT_PIN));
}


// turns LEDS off
void bothLedsOFF() 
{
	PORTD &= ~((1 << LED_LEFT_PIN) | (1 << LED_RIGHT_PIN));
}


//flashes SOS to signal battery is low
void SOS() 
{
	//reset LEDS
	bothLedsOFF();
	delay(300); //ms

	//dit dit dit
	for (byte i = 0;i < 3; i++)
	{
		bothLedsON();
		delay(300);
		bothLedsOFF();
		delay(300);
	}
	delay(700);//ms

	//dot    dot      dot
	for (byte i = 0; i < 3; i++)
	{
		bothLedsON();
		delay(1000);
		bothLedsOFF();
		delay(500);
	}
	delay(500);//ms
	
	//dit dit dit
	for (byte i = 0;i < 3; i++)
	{
		bothLedsON();
		delay(300);
		bothLedsOFF();
		delay(300);
	}
	delay(700);
}

////////////////////////////////////////////////////////////////////////////////////ATARI STUFF//////////////////////////////////////////////////////////////////////////

///atari button check procedure//
//
//(turned off by default)
//*
//void check_buttons(){                                           //check buttons method. the below lines check all the buttons and put them into an array (8 bin container)
// for (int i=0; i<NUMBUTTONS; i++){                               // we will do this untill we do once for every button we have available
//      buttons[i] = digitalRead(inputs[i]);                                                          //check button, throw it into a temporary variabe
//  
// //this is for sleep
//  if (buttons[i] == 0){ 
//             //unsigned long currentMillis = millis();
//          lastTimePressed = millis();
//  }
// }
// }
//*/
////

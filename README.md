# wirelessNES
Transmitter and Receiver firmware for a wireless NES controller and receiver which appears as a USB HID joystick in Windows.
written in Arduino C 

Author: Eason Smith (Eason@EasonRobotics.com)

For more documentation (HW and SW) on this project, please visit http://www.easonrobotics.com/?portfolio=wireless-nes-controller

**Acknowledgements**  
This project would not be possible without some amazing work done by the following individuals:
* Dean Cameron | LUFA USB Stack - http://fourwalledcubicle.com/LUFA.php
* Darran Hunt | Arduino Hacking Blog - http://hunt.net.nz/users/darran/
* Mark Feldman | ppl-pilot.com - http://www.ppl-pilot.com/
* Ryan97128 | Nintendo Controller MP3, Version 2.0 - http://www.instructables.com/id/Nintendo-Controller-MP3-Version-20/
* Mike McCauley | Virtual Wire Library - http://www.airspayce.com/mikem/arduino/VirtualWire/

**Authors Note:**  
2/8/2016 - While this project is no longer active (it's about 6 years old) I am currently working to document it for myself and others
who might find it interesting. Because of this, I cant currently make strong gaurentees that these files will still compile even for the original
arduino IDE version it was designed for (V0020). This is due to small code / comment cleanups here and there while the hardware has remained
in storage. I am actively working to confirm functionality using other methods, and I hope to reproduce misssing documentation as I go along. 
Thanks for your patience. -Eason

**Requires:**
* Arduino IDE V0020 (or below V1.0)
* Virtualwire Library - http://www.airspayce.com/mikem/arduino/VirtualWire/
* Arduino UNO rev2 with an onboard atmega8U2 (may work for atmega16U2, but this is not tested)

**What's currently missing:**  

1. 8U2 Hex file for the atmega 8U2:     Not missing actually. I have it on my PC but need to verify that it is the 
final (working) atmega8u2 hex file I used for this project. This will be available shortly. 

2. Schematic/Pinout Diagram:     I will most likely have to reproduce the full schematics of the project and controller pinout connections 
from scratch (ugg), however I did leave myself enough clues/comments in both the software I wrote and from old photos I took to be able
to accurately reproduce the schematic. This is also on the way.

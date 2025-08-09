# Wireless NES Controller

A modernized wireless NES controller that maintains the original look and feel while adding contemporary wireless functionality and USB compatibility for retro gaming on modern systems.

![Wireless NES Controller](https://img.shields.io/badge/Status-Retired%20Project-orange) ![Language](https://img.shields.io/badge/Language-C-blue) ![Platform](https://img.shields.io/badge/Platform-Arduino-green) ![License](https://img.shields.io/badge/License-MIT-lightgrey)

## 🎮 Project Overview

This project transforms a classic NES controller into a wireless, rechargeable gaming device compatible with modern computers. Unlike the original infrared wireless NES controllers that suffered from line-of-sight issues, this implementation uses 315MHz RF communication for reliable wireless gameplay.

### Key Features

- **Wireless Communication**: 315MHz RF Link operating at 4800 baud
- **USB HID Compatibility**: Appears as a standard joystick to Windows PCs
- **Rechargeable Battery**: Internal 3.7V LiPo with charging circuit
- **Power Management**: Automatic sleep mode and low battery detection
- **Visual Feedback**: Illuminated Nintendo logo with custom LED effects
- **Original Form Factor**: Fits entirely within the original NES controller case

## 🔧 Technical Specifications

### Hardware Components

**Transmitter (Controller)**
- **MCU**: Atmel ATmega328P with Arduino bootloader
- **Power**: 400mAh 3.7V LiPo battery (boosted to 5V)
- **Wireless**: 315MHz RF transmitter module
- **Features**: 
  - Physical on/off switch
  - 2.5mm charging port
  - 4-wire JST programming connector
  - Dual LED backlit Nintendo logo

**Receiver (Base Station)**
- **Platform**: Arduino Uno with custom 8U2 firmware
- **Wireless**: 315MHz RF receiver module
- **USB**: Custom HID descriptor (32-button joystick)
- **Communication**: UART to 8U2, USB HID to PC

### Software Architecture

- **Language**: C/Arduino
- **Libraries**: VirtualWire for reliable RF communication
- **Power Management**: Sleep modes, voltage monitoring, watchdog timers
- **Button Reading**: NES shift register protocol implementation
- **LED Effects**: PWM-controlled startup/sleep/low battery animations

## 🚀 Getting Started

### Prerequisites

- Arduino IDE
- Atmel FLIP software (for 8U2 firmware flashing)
- Basic soldering equipment
- Oscilloscope (for debugging RF communication)

### Hardware Assembly

1. **Controller Modification**
   - Remove internal plastic supports using a Dremel
   - Cut openings for power switch and programming connector
   - Install diffused plastic for LED light pipe under Nintendo logo

2. **Circuit Assembly**
   - Solder ATmega328P minimal circuit on perfboard
   - Install LiPo battery and boost regulator
   - Connect RF transmitter with ~6" antenna wire
   - Wire button matrix to microcontroller pins

3. **Base Station Setup**
   - Connect 315MHz receiver to Arduino Uno
   - Flash custom receiver firmware to ATmega328P
   - Flash custom HID firmware to 8U2 using Atmel FLIP

### Software Installation

1. **Transmitter Firmware**
   ```bash
   git clone https://github.com/EasonNYC/wirelessNES
   cd wirelessNES/transmitter
   # Load transmitter.ino in Arduino IDE and upload
   ```

2. **Receiver Firmware**
   ```bash
   cd wirelessNES/receiver
   # Load receiver.ino in Arduino IDE and upload to fresh ATmega328P
   ```

3. **8U2 HID Firmware**
   - Put Arduino Uno 8U2 into DFU mode
   - Use Atmel FLIP to flash custom HID hex file
   - *Note: This disables USB programming until original firmware is restored*

## 📡 Communication Protocol

The system uses a custom packet structure over 315MHz RF:

```c
typedef struct {
    uint8_t buttons;     // 8-bit button state
    uint8_t checksum;    // Error detection
    uint16_t sequence;   // Packet ordering
} controller_packet_t;
```

**Button Mapping** (NES Standard):
- Bit 0: A Button
- Bit 1: B Button  
- Bit 2: Select
- Bit 3: Start
- Bit 4: Up
- Bit 5: Down
- Bit 6: Left
- Bit 7: Right

## 🔋 Power Management

### Battery Life Optimization

- **Sleep Mode**: Automatically enters low-power state after 7 minutes of inactivity
- **Wake Mechanism**: Press Select button to wake from sleep
- **Low Battery Warning**: LED pulsing and automatic shutdown at 3.3V
- **Charging**: 2.5mm barrel jack with MAX1555-based LiPo charger

### LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| Solid On | Normal Operation |
| Slow Pulse | Low Battery Warning |
| Fast Blink | Entering Sleep Mode |
| Fade In/Out | Wake Up Sequence |

## 🛠️ Development Process

This project was developed as a learning exercise in embedded systems, representing one of my first major electronics projects. The methodical approach included:

1. **Requirements Analysis**: Defining goals and technical constraints
2. **Research Phase**: Understanding NES protocol, wireless options, USB HID
3. **Incremental Development**: Building from simple button reading to full wireless system
4. **Testing & Validation**: Measuring latency, reliability, and battery life

### Key Technical Challenges Solved

- **RF Reliability**: Overcame packet loss issues by implementing VirtualWire library
- **Space Constraints**: Designed minimal circuit to fit in original controller case
- **Power Efficiency**: Implemented sophisticated sleep/wake state machine
- **USB Compatibility**: Created custom HID descriptor for seamless PC integration

## 📊 Performance Metrics

- **Latency**: <50ms button press to PC recognition
- **Range**: ~30 feet through walls
- **Battery Life**: 8-12 hours continuous use
- **Reliability**: >99% packet success rate with VirtualWire

## 🎯 Future Improvements

While this project is officially retired, potential enhancements might include:

- **Bluetooth Low Energy**: Modern wireless standard for better compatibility
- **Multiple Controller Support**: Multiplayer gaming capability  
- **Universal Compatibility**: Support for other retro controller protocols (SNES, Genesis)
- **Custom PCB**: Professional board design for improved reliability
- **Haptic Feedback**: Rumble motor integration

## 🙏 Acknowledgments

This project was inspired by and builds upon work from several people:

- **Dean Cameron** - LUFA USB Stack
- **Mike McCauley** - VirtualWire Library  
- **Darran Hunt** - Arduino USB HID implementation
- **Mark Feldman** - NES controller modification techniques
- **Ryan97128** - Nintendo logo illumination method

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Note**: This project represents historical work from 2009-2012 during my transition into embedded systems engineering. While the code and techniques reflect the Arduino ecosystem of that era, the fundamental embedded systems concepts and problem-solving approaches remain relevant for modern firmware development.

*For questions about this project or my current embedded systems work, please connect with me on [LinkedIn](https://linkedin.com/in/easonsmith).*

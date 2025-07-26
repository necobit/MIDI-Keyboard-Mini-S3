# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MIDI keyboard project for the M5StampS3 microcontroller that creates a multi-octave keyboard interface with PWM audio output. The project implements both MIDI communication and direct audio synthesis.

## Development Commands

### Build and Upload
- **Build project**: `pio run`
- **Upload to device**: `pio run -t upload`
- **Build and upload**: `pio run -t upload`
- **Clean build**: `pio run -t clean`

### Monitoring and Debugging
- **Serial monitor**: `pio device monitor`
- **List connected devices**: `pio device list`

### Project Management
- **Check code**: `pio check` (static analysis)
- **Test**: `pio test` (if tests are implemented)

## Hardware Architecture

### M5StampS3 Pin Configuration
The project uses specific GPIO pins for different functions:

**MIDI I/O:**
- GPIO13 (MO): MIDI OUT via Serial1 TX
- GPIO1 (MI): MIDI IN via Serial1 RX

**Keyboard Matrix:**
- GPIO pins 1-10: Musical keys (KC, KC#, KD, KD#, KE, KF, KF#, KG, KG#, KA)
- GPIO pins 11-12: Additional keys (KA#, KB)
- All keys use pull-up resistors and ground when pressed

**Control Switches:**
- OU (Octave Up): Increases MIDI note by +12
- OD (Octave Down): Decreases MIDI note by -12
- CU, CD, S1-S3: Expansion switches for additional functionality

**Audio Output:**
- PWM: Direct audio frequency output corresponding to pressed keys

### MIDI Implementation
- Base note: KC = MIDI note 60 (Middle C)
- Note range: 0-127 with octave limiting
- Sends Note On when key pressed, Note Off when released
- Simultaneous PWM audio output at corresponding frequencies

## Code Structure

- `src/main.cpp`: Main application code (currently contains template)
- `platformio.ini`: PlatformIO configuration for M5StampS3
- `docs/StampS3.png`: Hardware pin diagram showing GPIO assignments

## Development Notes

The current `main.cpp` contains only template code. The actual MIDI keyboard implementation needs to be developed based on the Japanese specifications in the README.md, which describes:

1. Multi-key MIDI keyboard with chromatic layout
2. Octave shift controls (±12 semitones)
3. Dual output: MIDI messages + PWM audio
4. Note range limiting (0-127) with octave boundaries
5. Pull-up input configuration for all switches

The pin diagram in `docs/StampS3.png` shows the complete GPIO mapping for the M5StampS3 board.
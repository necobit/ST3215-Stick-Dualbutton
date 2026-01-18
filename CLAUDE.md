# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PlatformIO project for M5Stack StampS3 (ESP32-S3) using the Arduino framework. Controls 3x STS3215 servos with 6 buttons (2 per servo for CW/CCW rotation).

## Key Components

- **FTServo library**: Uses `SMS_STS` class for STS3215 communication
- **Serial1**: TX=GPIO43, RX=GPIO44, 1Mbps baud rate
- **Servo IDs**: 1, 2, 3
- **Button pins**: GPIO1-6 (INPUT_PULLUP, active LOW)

## Build Commands

```bash
# Build the project
pio run

# Upload to device
pio run --target upload

# Clean build files
pio run --target clean

# Monitor serial output
pio device monitor

# Build and upload in one command
pio run --target upload && pio device monitor
```

## Project Structure

- `src/main.cpp` - Main application code
- `include/` - Project header files
- `lib/` - Private libraries (compiled to static libraries)
- `platformio.ini` - PlatformIO configuration

## Hardware

- **Board**: M5Stack StampS3 (ESP32-S3)
- **Framework**: Arduino
- **Platform**: espressif32

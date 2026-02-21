# Arduino MAVLink Desktop Telemetry System

MAVLink-based telemetry system using LoRa wireless communication for real-time sensor monitoring.

## Overview

This project implements a complete telemetry system with:
- **Transmitter**: Arduino Uno or STM32F401 with multiple sensors
- **LoRa Communication**: LR900 modules at 57600 baud
- **Receiver**: Go application for desktop monitoring
- **Protocol**: MAVLink v2 (ArduPilot compatible)

## Features

- Real-time sensor data transmission at 10Hz
- Multi-sensor support (BMP280, AHT20, MPU6050/6500/9250)
- Temperature, humidity, pressure, altitude monitoring
- IMU data (accelerometer, gyroscope, magnetometer)
- Orientation tracking (roll, pitch, yaw)
- Fail-safe operation (works with missing sensors)

## Hardware Requirements

### Transmitter
- Arduino Uno or STM32F401CCU6 (WeAct Black Pill)
- LR900 LoRa module
- BMP280 (pressure/temperature sensor)
- AHT20 (humidity sensor)
- MPU6050/6500/9250 (IMU sensor)

### Receiver
- LR900 LoRa module
- USB to serial adapter

## Project Structure

```
arduino-mavlink-desktop/
├── arduino/
│   └── transmitter/          # Arduino Uno transmitter
│       ├── transmitter.ino   # Arduino sketch
│       └── README.md         # Arduino-specific docs
├── stm32/
│   └── transmitter/          # STM32 transmitter
│       ├── transmitter.ino   # STM32 sketch
│       └── README.md         # STM32-specific docs
├── receiver.go               # Go MAVLink receiver
└── README.md                 # This file
```

## Quick Start

### Arduino Transmitter
See [arduino/transmitter/README.md](arduino/transmitter/README.md)

### STM32 Transmitter
See [stm32/transmitter/README.md](stm32/transmitter/README.md)

### Receiver
```bash
# Run the receiver
go run receiver.go

# Expected output:
# Temp: 27.0°C | Humidity: 35.8% | Pressure: 973.1 hPa | Altitude: 339.3m
# Accel: (-0.09, -0.12, 11.27) m/s²
# Gyro: (-1.32, 1.09, -0.29) °/s
# Orientation: Roll=-0.6° Pitch=0.5° Yaw=-5.9°
```

## MAVLink Messages

The system transmits the following MAVLink messages:
1. `HEARTBEAT` - System status with counter
2. `SCALED_PRESSURE` - Pressure and temperature
3. `VFR_HUD` - Altitude data
4. `NAMED_VALUE_FLOAT` - Humidity readings
5. `SCALED_IMU2` - Accelerometer and gyroscope
6. `ATTITUDE` - Roll, pitch, yaw orientation

## Communication Settings

- **Baud Rate**: 57600
- **Update Rate**: 10Hz (100ms interval)
- **Protocol**: MAVLink v2

## Wiring

### Arduino Uno
```
LR900 LoRa:
  TX → D2 (RX)
  RX → D3 (TX)
  VCC → 5V
  GND → GND

I2C Sensors (A4=SDA, A5=SCL):
  BMP280: SDA, SCL, VCC (3.3V), GND
  AHT20: SDA, SCL, VCC (3.3V), GND
  MPU6050: SDA, SCL, VCC (3.3V), GND
```

### STM32F401CCU6
```
LR900 LoRa:
  TX → PA10 (RX)
  RX → PA9 (TX)
  VCC → 5V
  GND → GND

I2C Sensors (PB7=SDA, PB6=SCL):
  BMP280: SDA, SCL, VCC (3.3V), GND
  AHT20: SDA, SCL, VCC (3.3V), GND
  MPU6050/6500: SDA, SCL, VCC (3.3V), GND
```

## Troubleshooting

### No data received
1. Check LoRa connections
2. Verify baud rate is 57600
3. Check serial port in receiver.go
4. Ensure transmitter LED is blinking

### Sensor not found
The system is fail-safe and will continue with available sensors. Check:
1. I2C wiring (SDA/SCL)
2. Sensor power (3.3V)
3. Serial monitor for sensor status messages

### MPU sensor issues
- MPU9250 modules may actually be MPU6500 or MPU6050
- Code auto-detects chip type (WHO_AM_I register)
- Magnetometer only available on MPU9250/9255

## License

MIT

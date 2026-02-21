# Arduino Uno MAVLink Transmitter

MAVLink telemetry transmitter for Arduino Uno with multi-sensor support.

## Hardware

- **Board**: Arduino Uno
- **LoRa**: LR900 module
- **Sensors**: BMP280, AHT20, MPU6050

## Features

- MAVLink v2 protocol transmission
- 10Hz update rate (100ms interval)
- Multi-sensor data collection
- Fail-safe operation (continues with available sensors)
- Auto-detection of MPU chip type
- Real-time serial monitoring

## Wiring Diagram

### LoRa Module (LR900)
```
LR900 Pin    →  Arduino Pin
TX           →  D2 (RX)
RX           →  D3 (TX)
VCC          →  5V
GND          →  GND
```

### I2C Sensors
```
Sensor       →  Arduino Pin
All SDA      →  A4
All SCL      →  A5
All VCC      →  3.3V
All GND      →  GND
```

**Sensors on I2C bus:**
- BMP280 (0x76 or 0x77) - Pressure/Temperature
- AHT20 (0x38) - Humidity
- MPU6050 (0x68) - IMU (Accelerometer/Gyroscope)

## Libraries Required

Install via Arduino Library Manager:

```
- Adafruit BMP280 Library
- Adafruit AHTX0
- MPU6050_light
- MAVLink (custom - included in sketch)
```

## Upload Instructions

1. Open `transmitter.ino` in Arduino IDE
2. Select **Board**: Arduino Uno
3. Select **Port**: (your Arduino port)
4. Click **Upload**

## Serial Monitor Output

```
MAVLink Transmitter Starting...
AHT20 found!
BMP280 found!
MPU6050 found!
Calculating offsets, do not move MPU6050
MPU6050 ready!
Free RAM: 856 bytes
Setup complete! Sending MAVLink at 10Hz (100ms interval).

#10 R:-0.6° P:0.6° Y:-0.4° | T:27.0C H:35.8% Alt:339.3m
#20 R:-0.5° P:0.7° Y:-0.3° | T:27.1C H:35.9% Alt:339.4m
```

## Data Transmitted

### MAVLink Messages (10Hz rate)
1. **HEARTBEAT** - System alive with packet counter
2. **SCALED_PRESSURE** - Pressure (hPa) and temperature (°C)
3. **VFR_HUD** - Altitude (meters MSL)
4. **NAMED_VALUE_FLOAT** - Humidity (%)
5. **SCALED_IMU2** - Accelerometer (m/s²) and gyroscope (°/s)
6. **ATTITUDE** - Roll, pitch, yaw (radians)

### Sensor Readings
- **Temperature**: BMP280 (°C)
- **Humidity**: AHT20 (%)
- **Pressure**: BMP280 (hPa)
- **Altitude**: BMP280 calculated (meters)
- **Accelerometer**: MPU6050 (m/s²)
- **Gyroscope**: MPU6050 (°/s)
- **Orientation**: Roll, Pitch, Yaw (degrees)

## Configuration

### Baud Rates
```cpp
Serial.begin(9600);      // Debug serial (USB)
lora.begin(57600);       // LoRa serial (SoftwareSerial)
```

### Update Interval
```cpp
const long interval = 100;  // 100ms = 10Hz
```

### I2C Addresses
```cpp
BMP280: 0x76 (or 0x77)
AHT20:  0x38
MPU6050: 0x68
```

## Troubleshooting

### Sensor Not Found
**Symptom**: "Could not find [sensor]!" message loops forever

**Solution**:
1. Check sensor wiring (SDA → A4, SCL → A5)
2. Verify 3.3V power supply
3. Run I2C scanner to detect addresses
4. Check sensor solder connections

### MPU6050 Error Codes
- `0` - Success
- `1` - Sensor not found at 0x68
- `2` - Communication error

**Solution**:
1. Verify MPU6050 is at 0x68 (not 0x69)
2. Check I2C pullup resistors
3. Try different MPU6050 module

### Low Memory Warning
**Symptom**: Sketch uses >90% of dynamic memory

**Solution**:
- Already optimized with F() macros
- Uses 64-byte buffer (minimal size)
- Reduced debug output frequency (every 10th)

### No LoRa Transmission
**Symptom**: Serial monitor works but receiver shows nothing

**Solution**:
1. Check LR900 connections (TX→D2, RX→D3)
2. Verify 5V power to LR900
3. Confirm 57600 baud rate on receiver
4. Check LR900 configuration (should be transparent mode)

## Memory Usage

**Program**: ~26KB / 32KB (81%)
**Dynamic Memory**: ~856 bytes free / 2048 bytes (58% used)

Optimizations:
- F() macros for strings in PROGMEM
- 64-byte MAVLink buffer
- Minimal debug output
- No floating-point string conversions in loop

## Pin Summary

| Function | Arduino Pin | Notes |
|----------|-------------|-------|
| LoRa RX | D2 | SoftwareSerial receive |
| LoRa TX | D3 | SoftwareSerial transmit |
| I2C SDA | A4 | All sensors |
| I2C SCL | A5 | All sensors |
| USB Serial | D0/D1 | Debug output |

## Performance

- **Update Rate**: 10Hz (100ms interval)
- **LoRa Baud**: 57600
- **I2C Clock**: 100kHz (default)
- **Packet Size**: ~40 bytes per cycle (6 messages)
- **Throughput**: ~4000 bps

## Compatibility

- **Arduino IDE**: 1.8.x or 2.x
- **Board**: Arduino Uno, Nano, Pro Mini (ATmega328P)
- **MAVLink**: v2 (ArduPilot compatible)

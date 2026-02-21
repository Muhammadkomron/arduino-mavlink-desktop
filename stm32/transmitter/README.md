# STM32F401 MAVLink Transmitter

MAVLink telemetry transmitter for STM32F401CCU6 (WeAct Black Pill) with multi-sensor support.

## Hardware

- **Board**: STM32F401CCU6 (WeAct Black Pill)
- **LoRa**: LR900 module
- **Sensors**: BMP280, AHT20, MPU6050/6500/9250

## Features

- MAVLink v2 protocol transmission
- 10Hz update rate (100ms interval)
- Multi-sensor data collection
- Fail-safe operation (continues with available sensors)
- Auto-detection of MPU chip type (6050/6500/9250/9255)
- Magnetometer support (MPU9250/9255 only)
- Tilt-compensated yaw (with magnetometer)
- Gyro-integrated yaw (6-axis IMU)
- Raw I2C implementation (no library dependencies)
- Real-time serial monitoring

## Wiring Diagram

### LoRa Module (LR900)
```
LR900 Pin    →  STM32 Pin
TX           →  PA10 (RX1)
RX           →  PA9 (TX1)
VCC          →  5V
GND          →  GND
```

### I2C Sensors
```
Sensor       →  STM32 Pin
All SDA      →  PB7
All SCL      →  PB6
All VCC      →  3.3V
All GND      →  GND
```

**Sensors on I2C bus:**
- BMP280 (0x76 or 0x77) - Pressure/Temperature
- AHT20 (0x38) - Humidity
- MPU6050/6500/9250 (0x68) - IMU

## Libraries Required

Install via Arduino Library Manager:

```
- Adafruit BMP280 Library
- Adafruit AHTX0
- Wire (built-in)
- HardwareSerial (built-in)
```

**Note**: MPU sensor uses raw I2C (no library needed)

## Board Setup

### Arduino IDE Configuration

1. Install STM32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
   - Tools → Board → Boards Manager → Search "STM32" → Install

2. Board Settings:
   - **Board**: Generic STM32F4 series
   - **Board part number**: BlackPill F401CC
   - **Upload method**: STM32CubeProgrammer (DFU)
   - **USB support**: CDC (generic Serial)
   - **U(S)ART support**: Enabled (generic Serial)

## Upload Instructions

### Using DFU Mode (Recommended)

1. Enter DFU mode:
   - Hold **BOOT0** button
   - Press and release **NRST** button
   - Release **BOOT0** button

2. Verify DFU mode:
   ```bash
   # macOS/Linux
   lsusb | grep DFU
   # Should show: STM32 BOOTLOADER
   ```

3. Upload:
   - Select **Upload method**: STM32CubeProgrammer (DFU)
   - Click **Upload** in Arduino IDE
   - Board will auto-reset after upload

4. Exit DFU mode:
   - Press **NRST** button
   - Or unplug/replug USB

### Using ST-Link (Alternative)
1. Connect ST-Link to STM32 (SWDIO, SWCLK, GND, 3.3V)
2. Select **Upload method**: STLink
3. Click **Upload**

## Serial Monitor Output

```
=== STM32 MAVLink Transmitter ===

Starting sensor initialization...

BMP280: OK (0x77)
AHT20: OK
MPU: Detected MPU6500 (0x70)
MPU: 6-axis sensor (no magnetometer)
MPU: Initialized successfully

All sensors ready!
I2C Speed: 100 kHz
LoRa: 57600 baud on Serial1 (PA9/PA10)

Sending MAVLink at 10Hz...

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
- **Accelerometer**: MPU (m/s²)
- **Gyroscope**: MPU (°/s)
- **Magnetometer**: MPU9250/9255 only (μT)
- **Orientation**: Roll, Pitch, Yaw (degrees)

## Configuration

### Baud Rates
```cpp
Serial.begin(115200);        // Debug serial (USB)
Serial1.begin(57600);        // LoRa serial (Hardware UART)
```

### Update Interval
```cpp
const long interval = 100;   // 100ms = 10Hz
```

### I2C Configuration
```cpp
Wire.setSDA(PB7);
Wire.setSCL(PB6);
Wire.setClock(100000);       // 100kHz for stability
```

### I2C Addresses
```cpp
BMP280: 0x76 (or 0x77)
AHT20:  0x38
MPU:    0x68
AK8963: 0x0C (magnetometer inside MPU9250)
```

## MPU Sensor Auto-Detection

The code automatically detects the MPU chip type:

| WHO_AM_I | Chip | Features |
|----------|------|----------|
| 0x68 | MPU6050 | 6-axis (accel + gyro) |
| 0x70 | MPU6500 | 6-axis (accel + gyro) |
| 0x71 | MPU9250 | 9-axis (accel + gyro + mag) |
| 0x73 | MPU9255 | 9-axis (accel + gyro + mag) |

### Yaw Calculation
- **9-axis (MPU9250/9255)**: Tilt-compensated magnetometer yaw
- **6-axis (MPU6050/6500)**: Gyroscope-integrated yaw

## Troubleshooting

### Sensor Not Found
**Symptom**: "BMP280: FAIL" or "AHT20: FAIL" message

**Solution**:
1. Check sensor wiring (SDA → PB7, SCL → PB6)
2. Verify 3.3V power supply
3. Check I2C pullup resistors (4.7kΩ recommended)
4. Try different I2C address for BMP280 (0x76 or 0x77)

### MPU Sensor Issues
**Symptom**: "MPU: FAIL" or "MPU: Unknown chip ID"

**Solution**:
1. Many "MPU9250" modules are actually MPU6050 or MPU6500
2. Code auto-detects chip type - check serial output
3. Verify I2C wiring and 3.3V power
4. Slow I2C clock helps: `Wire.setClock(100000);`

### Magnetometer Not Working
**Symptom**: "MPU: No magnetometer available"

**Solution**:
- Only MPU9250/9255 have magnetometers
- MPU6050/6500 are 6-axis only (no magnetometer)
- System will use gyro-integrated yaw instead (works fine)

### Upload Failed
**Symptom**: "No DFU capable USB device available"

**Solution**:
1. Enter DFU mode properly (hold BOOT0, press NRST, release BOOT0)
2. Check USB cable (must support data)
3. Install STM32CubeProgrammer
4. Try different USB port
5. Use ST-Link programmer instead

### Board Not Recognized
**Symptom**: Serial port not showing up

**Solution**:
1. Press **NRST** button to exit DFU mode
2. Check **USB support** is set to **CDC**
3. Install STM32 VCP drivers (Windows)
4. Unplug/replug USB cable

### I2C Bus Hangs
**Symptom**: Sensors freeze or don't respond

**Solution**:
1. Initialize MPU first (most sensitive to bus conditions)
2. Add delays between sensor initializations (200ms)
3. Use slower I2C clock (100kHz instead of 400kHz)
4. Check for multiple sensors with same address

### No LoRa Transmission
**Symptom**: Serial monitor works but receiver shows nothing

**Solution**:
1. Check LR900 connections (TX→PA10, RX→PA9)
2. Verify 5V power to LR900
3. Confirm 57600 baud rate on receiver
4. Ensure **U(S)ART support** is enabled in board settings
5. Check LR900 configuration (should be transparent mode)

## Memory Usage

**Flash**: ~45KB / 256KB (18%)
**SRAM**: ~12KB free / 64KB (81% free)

Advantages over Arduino:
- More flash memory (256KB vs 32KB)
- More SRAM (64KB vs 2KB)
- Faster CPU (84MHz vs 16MHz)
- Hardware UART for LoRa (no SoftwareSerial)
- More stable I2C with multiple sensors

## Pin Summary

| Function | STM32 Pin | Notes |
|----------|-----------|-------|
| LoRa RX | PA10 | Hardware UART1 |
| LoRa TX | PA9 | Hardware UART1 |
| I2C SDA | PB7 | All sensors |
| I2C SCL | PB6 | All sensors |
| USB D+ | PA12 | Debug serial |
| USB D- | PA11 | Debug serial |
| BOOT0 | BOOT0 | DFU mode button |
| NRST | NRST | Reset button |

## Performance

- **Update Rate**: 10Hz (100ms interval)
- **LoRa Baud**: 57600 (Hardware UART)
- **I2C Clock**: 100kHz (for stability)
- **CPU Speed**: 84MHz
- **Packet Size**: ~40 bytes per cycle (6 messages)
- **Throughput**: ~4000 bps

## Raw I2C Implementation

Unlike Arduino version, STM32 code uses raw I2C register access:

**Advantages**:
- No library conflicts with multiple sensors
- Lighter memory footprint
- More reliable on shared I2C bus
- Better control over timing

**Registers Used**:
- `0x75` - WHO_AM_I (chip identification)
- `0x6B` - PWR_MGMT_1 (power management)
- `0x3B` - ACCEL_XOUT_H (accelerometer data)
- `0x43` - GYRO_XOUT_H (gyroscope data)
- `0x41` - TEMP_OUT_H (temperature)
- `0x37` - INT_PIN_CFG (magnetometer access)
- `0x0C` - AK8963 magnetometer (MPU9250 only)

## Compatibility

- **Arduino IDE**: 1.8.x or 2.x
- **Board Package**: STM32duino
- **Board**: STM32F401CCU6 (WeAct Black Pill)
- **MAVLink**: v2 (ArduPilot compatible)
- **STM32CubeProgrammer**: Required for DFU upload

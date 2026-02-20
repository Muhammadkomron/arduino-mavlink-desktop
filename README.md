# Arduino MAVLink LoRa Communication System

This project implements a wireless sensor data transmission system using Arduino Nano, LR900 LoRa module, and MAVLink protocol.

## Hardware Components

- **Arduino Nano**: Microcontroller
- **LR900**: LoRa wireless module (9600 baud)
- **AHT20**: Temperature and humidity sensor (I2C)
- **BMP280**: Pressure and altitude sensor (I2C)

## Wiring Connections

### Arduino Nano Pin Connections:
- **AHT20**:
  - SDA → A4
  - SCL → A5
  - VCC → 3.3V/5V
  - GND → GND

- **BMP280**:
  - SDA → A4 (shared with AHT20)
  - SCL → A5 (shared with AHT20)
  - VCC → 3.3V
  - GND → GND

- **LR900 LoRa Module**:
  - TX → D3 (Arduino RX)
  - RX → D2 (Arduino TX)
  - VCC → 5V
  - GND → GND

## Project Structure

```
arduino-mavlink-desktop/
│
├── transmitter/
│   └── transmitter.ino      # Arduino sketch for data transmission
│
├── receiver.go               # Go receiver application
├── go.mod                    # Go module dependencies
└── README.md                # This file
```

## Transmitter Setup (Arduino)

1. **Install Arduino Libraries**:
   ```bash
   # In Arduino IDE, install these libraries:
   - Adafruit AHTX0 (via Library Manager)
   - Adafruit BMP280 (via Library Manager)
   ```

2. **Sensor Configuration**:
   - The sketch automatically tries BMP280 at addresses 0x76 and 0x77
   - For combined AHT20+BMP280 modules, the BMP280 is typically at 0x77
   - If sensors aren't detected, the sketch will run an I2C scan to show available addresses

3. **Upload the Sketch**:
   - Open `transmitter/transmitter.ino` in Arduino IDE
   - Select "Arduino Nano" as board
   - Select correct COM port
   - Upload the sketch

## Receiver Setup (Go)

1. **Install Dependencies**:
   ```bash
   go mod download
   ```

2. **Run the Receiver**:
   ```bash
   go run receiver.go
   ```

3. **Select Serial Port**:
   - The application will list all available ports with details
   - Enter the port number or full path where LR900 receiver is connected
   - The receiver will start displaying data in real-time

## Data Transmitted

The system transmits the following sensor data using MAVLink protocol:

- **Temperature** (°C) - from AHT20 and BMP280
- **Humidity** (%) - from AHT20
- **Pressure** (hPa) - from BMP280
- **Altitude** (m) - calculated from pressure
- **Package Count** - incremental counter for each transmission

Data is sent every 2 seconds via LoRa at 57600 baud.

## Data Protocol

Due to memory limitations on Arduino Nano (2KB RAM), the system uses a simple text protocol:

**Format**: `$DATA,package_count,temperature,humidity,pressure,altitude*`

Example: `$DATA,1,25.50,45.20,1013.25,10.50*`

Fields:
- package_count: Incremental counter
- temperature: Celsius (2 decimal places)
- humidity: Percentage (2 decimal places)
- pressure: hPa (2 decimal places)
- altitude: meters (2 decimal places)

## LR900 Configuration

The LR900 modules should be pre-configured using the official desktop software. Ensure both transmitter and receiver modules are configured with:
- Same frequency/channel
- Same data rate (57600 baud)
- Same address or broadcast mode
- Appropriate power settings for your range requirements

## Troubleshooting

### Sensor Not Found
- Check I2C connections (SDA/SCL)
- Verify sensor power supply
- Try alternative I2C addresses

### No Data Received
- Verify LR900 module connections
- Check if both modules are on the same channel
- Ensure correct baud rate (9600)
- Check antenna connections

### Serial Port Not Found
- Install appropriate drivers for your USB-Serial adapter
- Check device manager (Windows) or /dev/ (Linux/Mac)
- Try different USB ports

## Serial Monitor Output

The Arduino transmitter outputs debug information via Serial Monitor (9600 baud):
- Sensor readings
- Package count
- Connection status

## License

This project is for educational purposes. Modify as needed for your application.
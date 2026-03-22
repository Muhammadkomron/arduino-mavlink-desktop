# NazarX Ground Station

CanSat telemetry system with MAVLink protocol, LoRa wireless communication, and a desktop ground station for real-time monitoring.

## Overview

- **Transmitters**: Arduino Uno or STM32F401 with BMP280, AHT20, MPU6050 sensors
- **Communication**: LR900 LoRa modules, MAVLink v2 protocol, 57600 baud
- **Ground Station**: Wails desktop app (Go + React) with real-time dashboard
- **Receiver**: STM32F401 with SD card logging (standalone option)

## Project Structure

```
arduino-mavlink-desktop/
├── desktop/                        # Ground station desktop app
│   ├── main.go                     # Wails entry point (fullscreen launch)
│   ├── wails.json                  # Wails configuration
│   ├── go.mod / go.sum             # Go dependencies
│   ├── build.sh                    # Cross-platform build script
│   ├── backend/
│   │   └── app.go                  # Go backend (serial, MAVLink, telemetry)
│   ├── frontend/
│   │   ├── src/
│   │   │   ├── App.jsx             # Main app with splash screen
│   │   │   ├── components/
│   │   │   │   ├── Header.jsx      # Title bar, theme toggle, mission timer
│   │   │   │   ├── Sidebar.jsx     # Connection, telemetry, GPS panels
│   │   │   │   ├── Commands.jsx    # Command buttons (telemetry, sim, cal)
│   │   │   │   ├── ChartCard.jsx   # Real-time sensor charts
│   │   │   │   ├── Rocket3D.jsx    # 3D orientation visualization
│   │   │   │   ├── GPSMap.jsx      # Leaflet GPS tracking map
│   │   │   │   ├── VideoFeed.jsx   # Camera feed
│   │   │   │   └── DataFlow.jsx    # Telemetry data log
│   │   │   └── __tests__/          # Frontend tests
│   │   ├── public/                 # Logos, flag assets
│   │   └── wailsjs/               # Auto-generated Wails bindings
│   ├── tests/
│   │   ├── run.sh                  # Test runner (all suites)
│   │   └── backend/
│   │       ├── unit/               # 23 unit tests
│   │       ├── integration/        # 4 integration tests
│   │       └── e2e/                # 5 e2e tests (with race detector)
│   └── build/
│       ├── appicon.png             # App icon
│       ├── darwin/                 # macOS build config
│       └── windows/                # Windows build config + NSIS installer
├── arduino/
│   └── transmitter/                # Arduino Uno transmitter
│       ├── transmitter.ino
│       └── README.md
├── stm32/
│   ├── transmitter/                # STM32 transmitter
│   │   ├── transmitter.ino
│   │   └── README.md
│   └── receiver/                   # STM32 receiver with SD logging
│       └── receiver.ino
└── README.md
```

## Ground Station

### Features

- Real-time charts for altitude, voltage, pressure, temperature, orientation
- 3D rocket model with roll/pitch/yaw visualization
- GPS tracking on Leaflet map
- Video feed from connected camera
- Command panel (telemetry on/off, simulation mode, calibrate, set time/date)
- Telemetry data flow log
- Dark/light theme with persistent preference
- Splash screen on launch
- Packet loss tracking

### Prerequisites

- [Go 1.22+](https://go.dev/dl/)
- [Node.js 18+](https://nodejs.org/)
- [Wails v2](https://wails.io/): `go install github.com/wailsapp/wails/v2/cmd/wails@latest`

### Development

```bash
cd desktop
wails dev
```

### Build

```bash
cd desktop

# Build for current platform
./build.sh mac

# All options
./build.sh mac          # macOS (current arch)
./build.sh mac-arm64    # macOS Apple Silicon
./build.sh mac-amd64    # macOS Intel
./build.sh mac-universal # macOS Universal binary
./build.sh windows      # Windows amd64
./build.sh linux        # Linux amd64
./build.sh all          # All platforms
```

Output goes to `desktop/build/bin/`.

### Tests

```bash
cd desktop
bash tests/run.sh
```

**110 tests total:**
- Go unit tests (23) — app initialization, connection, commands, message processing
- Go integration tests (4) — full telemetry pipeline, packet loss, sensor overwrite
- Go e2e tests (5) — session lifecycle, concurrency with race detector
- Frontend unit tests (65) — all components with Vitest + React Testing Library
- Frontend integration tests (6) — dashboard event handling
- Frontend e2e tests (6) — full connect → telemetry → command → disconnect flow

## Transmitters

### Arduino Uno
See [arduino/transmitter/README.md](arduino/transmitter/README.md)

### STM32F401
See [stm32/transmitter/README.md](stm32/transmitter/README.md)

## STM32 Receiver (SD Card Logging)

1. Upload `stm32/receiver/receiver.ino` to STM32F401
2. Insert FAT32-formatted microSD card
3. Power on — data logs to `LOG0.CSV`, `LOG1.CSV`, etc.
4. Monitor via USB serial at 115200 baud

## MAVLink Messages

| Message | Data |
|---------|------|
| `HEARTBEAT` | System status, packet counter |
| `SCALED_PRESSURE` | Pressure (hPa), temperature (°C) |
| `VFR_HUD` | Altitude (m) |
| `SYS_STATUS` | Battery voltage (V) |
| `NAMED_VALUE_FLOAT` | Humidity (%) |
| `SCALED_IMU2` | Accelerometer, gyroscope |
| `ATTITUDE` | Roll, pitch, yaw (rad → °) |
| `GPS_RAW_INT` | Lat, lon, alt, satellites, fix type |

## Hardware

### Sensors
- **BMP280** — pressure, temperature, altitude
- **AHT20** — humidity
- **MPU6050/6500/9250** — accelerometer, gyroscope (auto-detected)

### Wiring

**Arduino Uno:**
```
LR900: TX→D2, RX→D3, VCC→5V, GND
I2C (A4=SDA, A5=SCL): BMP280, AHT20, MPU6050
```

**STM32F401 Transmitter:**
```
LR900 (UART1): TX→PA10, RX→PA9, VCC→5V, GND
I2C (PB7=SDA, PB6=SCL): BMP280, AHT20, MPU6050
```

**STM32F401 Receiver:**
```
LR900 (UART1): TX→PA10, RX→PA9, VCC→5V, GND
SD Card (SPI1): CS→PA4, CLK→PA5, MISO→PA6, MOSI→PA7, VCC→3V3, GND
LED: PC13 (blinks on data)
```

## Troubleshooting

| Issue | Fix |
|-------|-----|
| No data received | Check LoRa wiring, verify 57600 baud, check serial port |
| Sensor not found | System is fail-safe — check I2C wiring and 3.3V power |
| MPU variant mismatch | Auto-detected via WHO_AM_I register |
| SD card not detected | Format FAT32, check SPI wiring, try different card |
| macOS app won't open | Right-click → Open, or `xattr -cr /path/to/app` |

## License

MIT

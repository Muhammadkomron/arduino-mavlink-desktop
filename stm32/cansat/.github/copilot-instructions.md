# Copilot instructions for `stm32-cansat`

## Build, test, and lint commands

This repository is a PlatformIO project with `transmitter` (flight firmware), `receiver` (ground firmware), and `native` (host protocol tests).

```bash
# Build
pio run -e transmitter
pio run -e receiver

# Flash over DFU (uses platformio.ini upload_command + firmware.bin)
pio run -e transmitter -t upload
pio run -e receiver -t upload
```

```bash
# Protocol tests on host
pio test -e native

# Run a single test folder
pio test -e native -f test_protocol
```

```bash
# Lint / static analysis
pio check -e transmitter
pio check -e receiver
```

## High-level architecture

- `src/main.cpp` only selects app role by compile-time flag:
  - `MODE_TRANSMITTER` -> `FlightApp` (`src/flight/flight_app.cpp`)
  - `MODE_RECEIVER` -> `GroundApp` (`src/ground/ground_app.cpp`)
- `FlightApp` pipeline (1 Hz):
  - reads MPU9250 (`src/common/sensors/imu_mpu9250.cpp`, I2C PD12/PD13),
  - reads INA219 (`src/common/sensors/power_ina219.cpp`, I2C PB6/PB7),
  - continuously parses GPS NMEA (`src/common/sensors/gps_nmea.cpp`, UART PA0/PA1),
  - builds fixed binary telemetry packet (`src/common/protocol/telemetry_packet.cpp`),
  - sends via XBee API frame transport (`src/common/link/xbee_api.cpp`, UART PA9/PA10).
- `GroundApp` receives XBee API frames, validates telemetry packet CRC, decodes packet, and reports sequence gaps (lost packets).
- `extra_script.py` generates `firmware.bin` post-build for DFU upload.

## Key conventions for edits

- Select device role through `platformio.ini` build flags (`MODE_TRANSMITTER` / `MODE_RECEIVER`), not runtime branching.
- Keep pin mapping in `platformio.ini` build flags and `include/cansat/board_config.hpp`; do not hardcode pin numbers in app logic.
- Telemetry over air must stay binary and fixed-size (`kTelemetryPacketSize = 50`) with little-endian field order and CRC16-CCITT in the last 2 bytes.
- Do not send floats over radio: packet fields are scaled integers (`*_e7`, `*_cm`, `*_dps10`, etc.).
- BME280 is intentionally disabled: preserve `STATUS_BME280_DISABLED` behavior unless hardware status changes.
- XBee link uses API framing (`0x10` TX request, `0x90` RX packet, `0x8B` TX status); keep this transport stable across flight and ground sides.
- Treat `.vscode/c_cpp_properties.json` and `.vscode/launch.json` as generated files; update `platformio.ini` instead.

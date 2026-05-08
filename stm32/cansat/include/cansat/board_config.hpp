#pragma once

#include <Arduino.h>
#include <cstdint>

#ifndef XBEE_RX_PIN
#define XBEE_RX_PIN PA10
#endif

#ifndef XBEE_TX_PIN
#define XBEE_TX_PIN PA9
#endif

#ifndef GPS_RX_PIN
#define GPS_RX_PIN PA1
#endif

#ifndef GPS_TX_PIN
#define GPS_TX_PIN PA0
#endif

#ifndef GPS_BAUD
#define GPS_BAUD 9600UL
#endif

#ifndef IMU_SCL_PIN
#define IMU_SCL_PIN PD12
#endif

#ifndef IMU_SDA_PIN
#define IMU_SDA_PIN PD13
#endif

#ifndef INA_SCL_PIN
#define INA_SCL_PIN PB6
#endif

#ifndef INA_SDA_PIN
#define INA_SDA_PIN PB7
#endif

#ifndef SERVO1_PIN
#define SERVO1_PIN PE9
#endif

#ifndef SERVO2_PIN
#define SERVO2_PIN PE11
#endif

#ifndef SERVO3_PIN
#define SERVO3_PIN PE13
#endif

#ifndef SERVO4_PIN
#define SERVO4_PIN PE14
#endif

#ifndef XBEE_DEST_ADDR64_HI
#define XBEE_DEST_ADDR64_HI 0x00000000UL
#endif

#ifndef XBEE_DEST_ADDR64_LO
#define XBEE_DEST_ADDR64_LO 0x0000FFFFUL
#endif

namespace cansat::config {

constexpr uint32_t kDebugBaud = 115200U;
constexpr uint32_t kXBeeBaud = 115200U;
constexpr uint32_t kGpsBaud = GPS_BAUD;
constexpr uint32_t kTelemetryPeriodMs = 1000U;
constexpr uint64_t kXBeeDestination64 =
    (static_cast<uint64_t>(XBEE_DEST_ADDR64_HI) << 32ULL) | static_cast<uint64_t>(XBEE_DEST_ADDR64_LO);

constexpr uint32_t kXBeeRxPin = XBEE_RX_PIN;
constexpr uint32_t kXBeeTxPin = XBEE_TX_PIN;
constexpr uint32_t kGpsRxPin = GPS_RX_PIN;
constexpr uint32_t kGpsTxPin = GPS_TX_PIN;
constexpr uint32_t kImuSclPin = IMU_SCL_PIN;
constexpr uint32_t kImuSdaPin = IMU_SDA_PIN;
constexpr uint32_t kInaSclPin = INA_SCL_PIN;
constexpr uint32_t kInaSdaPin = INA_SDA_PIN;

constexpr uint32_t kLedOk = PB0;
constexpr uint32_t kLedTx = PE1;
constexpr uint32_t kLedErr = PB14;

}  // namespace cansat::config

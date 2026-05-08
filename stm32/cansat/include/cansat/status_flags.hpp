#pragma once

#include <cstdint>

namespace cansat {

enum StatusFlag : uint16_t {
  STATUS_IMU_OK = 1U << 0,
  STATUS_MAG_OK = 1U << 1,
  STATUS_GPS_FIX = 1U << 2,
  STATUS_INA_OK = 1U << 3,
  STATUS_XBEE_DELIVERY_OK = 1U << 4,
  STATUS_BME280_DISABLED = 1U << 5,
  STATUS_GPS_FRESH = 1U << 6,
  STATUS_IMU_FRESH = 1U << 7,
  STATUS_INA_FRESH = 1U << 8,
};

}  // namespace cansat


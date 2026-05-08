#pragma once

#include <cstdint>

namespace cansat {

struct ImuData {
  int16_t ax_mg = 0;
  int16_t ay_mg = 0;
  int16_t az_mg = 0;
  int16_t gx_dps10 = 0;
  int16_t gy_dps10 = 0;
  int16_t gz_dps10 = 0;
  int16_t mx_ut10 = 0;
  int16_t my_ut10 = 0;
  int16_t mz_ut10 = 0;
  bool mag_ok = false;
};

struct PowerData {
  uint16_t bus_mv = 0;
  int16_t current_ma = 0;
};

struct GpsData {
  int32_t lat_e7 = 0;
  int32_t lon_e7 = 0;
  int32_t alt_cm = 0;
  int16_t speed_cms = 0;
  int16_t course_cdeg = 0;
  bool fix = false;
  bool fresh = false;
};

}  // namespace cansat


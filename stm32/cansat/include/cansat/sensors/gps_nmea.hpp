#pragma once

#include <Arduino.h>

#include "cansat/data_models.hpp"

namespace cansat {

class GpsNmea {
 public:
  bool begin(HardwareSerial& serial, uint32_t baud);
  void poll();
  GpsData latest() const;

 private:
  HardwareSerial* serial_ = nullptr;
  GpsData latest_{};
  char line_buf_[128]{};
  uint8_t line_len_ = 0;
  uint32_t last_update_ms_ = 0;

  void parse_line(char* line);
  void parse_gga(char* line);
  void parse_rmc(char* line);

  static int32_t parse_coord_e7(const char* value, char hemi);
  static int32_t parse_decimal_scaled(const char* value, int32_t scale);
};

}  // namespace cansat


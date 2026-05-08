#pragma once

#include <Wire.h>

#include "cansat/data_models.hpp"

namespace cansat {

class PowerIna219 {
 public:
  bool begin(TwoWire& bus);
  bool read(PowerData& out);

 private:
  static constexpr uint8_t kAddr = 0x40;
  static constexpr int32_t kShuntMilliohm = 100;  // typical module value

  TwoWire* bus_ = nullptr;

  bool write_u16(uint8_t reg, uint16_t value) const;
  bool read_u16(uint8_t reg, uint16_t& value) const;
};

}  // namespace cansat


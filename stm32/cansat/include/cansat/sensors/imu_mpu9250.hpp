#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "cansat/data_models.hpp"

namespace cansat {

class ImuMpu9250 {
 public:
  bool begin(TwoWire& bus);
  bool read(ImuData& out);

 private:
  static constexpr uint8_t kMpuAddr = 0x68;
  static constexpr uint8_t kMagAddr = 0x0C;

  TwoWire* bus_ = nullptr;

  bool write_reg(uint8_t addr, uint8_t reg, uint8_t value) const;
  bool read_regs(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) const;
};

}  // namespace cansat


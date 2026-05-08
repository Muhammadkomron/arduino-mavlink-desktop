#include "cansat/sensors/power_ina219.hpp"

namespace cansat {

bool PowerIna219::write_u16(uint8_t reg, uint16_t value) const {
  bus_->beginTransmission(kAddr);
  bus_->write(reg);
  bus_->write(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  bus_->write(static_cast<uint8_t>(value & 0xFFU));
  return bus_->endTransmission() == 0;
}

bool PowerIna219::read_u16(uint8_t reg, uint16_t& value) const {
  bus_->beginTransmission(kAddr);
  bus_->write(reg);
  if (bus_->endTransmission(false) != 0) {
    return false;
  }
  if (bus_->requestFrom(static_cast<int>(kAddr), 2) != 2) {
    return false;
  }
  const uint8_t msb = static_cast<uint8_t>(bus_->read());
  const uint8_t lsb = static_cast<uint8_t>(bus_->read());
  value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8U) | lsb);
  return true;
}

bool PowerIna219::begin(TwoWire& bus) {
  bus_ = &bus;
  // BRNG=32V, PG=320mV, BADC=12bit, SADC=12bit, MODE=shunt+bus continuous
  return write_u16(0x00, 0x399FU);
}

bool PowerIna219::read(PowerData& out) {
  if (bus_ == nullptr) {
    return false;
  }

  uint16_t bus_reg = 0;
  uint16_t shunt_reg = 0;
  if (!read_u16(0x02, bus_reg) || !read_u16(0x01, shunt_reg)) {
    return false;
  }

  const uint16_t bus_raw = static_cast<uint16_t>(bus_reg >> 3U);
  out.bus_mv = static_cast<uint16_t>(bus_raw * 4U);

  const int16_t shunt_raw = static_cast<int16_t>(shunt_reg);
  const int32_t shunt_uv = static_cast<int32_t>(shunt_raw) * 10;
  out.current_ma = static_cast<int16_t>(shunt_uv / kShuntMilliohm);
  return true;
}

}  // namespace cansat


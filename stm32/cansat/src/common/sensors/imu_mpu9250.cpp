#include "cansat/sensors/imu_mpu9250.hpp"

namespace cansat {

namespace {

inline int16_t to_i16(uint8_t msb, uint8_t lsb) {
  return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
}

}  // namespace

bool ImuMpu9250::write_reg(uint8_t addr, uint8_t reg, uint8_t value) const {
  bus_->beginTransmission(addr);
  bus_->write(reg);
  bus_->write(value);
  return bus_->endTransmission() == 0;
}

bool ImuMpu9250::read_regs(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) const {
  bus_->beginTransmission(addr);
  bus_->write(reg);
  if (bus_->endTransmission(false) != 0) {
    return false;
  }

  const int read_len = bus_->requestFrom(static_cast<int>(addr), static_cast<int>(len));
  if (read_len != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    data[i] = static_cast<uint8_t>(bus_->read());
  }
  return true;
}

bool ImuMpu9250::begin(TwoWire& bus) {
  bus_ = &bus;

  if (!write_reg(kMpuAddr, 0x6B, 0x00)) {  // wake up
    return false;
  }
  delay(50);

  // accel +-4g (8192 LSB/g), gyro +-1000 dps (32.8 LSB/dps)
  if (!write_reg(kMpuAddr, 0x1C, 0x08) || !write_reg(kMpuAddr, 0x1B, 0x10)) {
    return false;
  }

  // enable bypass mode for AK8963 magnetometer
  if (!write_reg(kMpuAddr, 0x37, 0x02)) {
    return false;
  }
  delay(10);

  // power-down then continuous measurement mode 2 (100 Hz, 16-bit)
  if (!write_reg(kMagAddr, 0x0A, 0x00)) {
    return false;
  }
  delay(10);
  if (!write_reg(kMagAddr, 0x0A, 0x16)) {
    return false;
  }
  delay(10);
  return true;
}

bool ImuMpu9250::read(ImuData& out) {
  if (bus_ == nullptr) {
    return false;
  }

  uint8_t raw[14]{};
  if (!read_regs(kMpuAddr, 0x3B, raw, sizeof(raw))) {
    return false;
  }

  const int16_t ax = to_i16(raw[0], raw[1]);
  const int16_t ay = to_i16(raw[2], raw[3]);
  const int16_t az = to_i16(raw[4], raw[5]);
  const int16_t gx = to_i16(raw[8], raw[9]);
  const int16_t gy = to_i16(raw[10], raw[11]);
  const int16_t gz = to_i16(raw[12], raw[13]);

  out.ax_mg = static_cast<int16_t>((static_cast<int32_t>(ax) * 1000) / 8192);
  out.ay_mg = static_cast<int16_t>((static_cast<int32_t>(ay) * 1000) / 8192);
  out.az_mg = static_cast<int16_t>((static_cast<int32_t>(az) * 1000) / 8192);

  out.gx_dps10 = static_cast<int16_t>((static_cast<int32_t>(gx) * 100) / 328);
  out.gy_dps10 = static_cast<int16_t>((static_cast<int32_t>(gy) * 100) / 328);
  out.gz_dps10 = static_cast<int16_t>((static_cast<int32_t>(gz) * 100) / 328);

  uint8_t st1 = 0;
  if (!read_regs(kMagAddr, 0x02, &st1, 1) || (st1 & 0x01U) == 0U) {
    out.mag_ok = false;
    out.mx_ut10 = 0;
    out.my_ut10 = 0;
    out.mz_ut10 = 0;
    return true;
  }

  uint8_t mag[7]{};
  if (!read_regs(kMagAddr, 0x03, mag, sizeof(mag))) {
    out.mag_ok = false;
    return true;
  }
  if ((mag[6] & 0x08U) != 0U) {  // magnetic sensor overflow
    out.mag_ok = false;
    return true;
  }

  const int16_t mx = to_i16(mag[1], mag[0]);
  const int16_t my = to_i16(mag[3], mag[2]);
  const int16_t mz = to_i16(mag[5], mag[4]);

  // AK8963 sensitivity is 0.15 uT/LSB in 16-bit mode.
  out.mx_ut10 = static_cast<int16_t>((static_cast<int32_t>(mx) * 3) / 2);
  out.my_ut10 = static_cast<int16_t>((static_cast<int32_t>(my) * 3) / 2);
  out.mz_ut10 = static_cast<int16_t>((static_cast<int32_t>(mz) * 3) / 2);
  out.mag_ok = true;
  return true;
}

}  // namespace cansat


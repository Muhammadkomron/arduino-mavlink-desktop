#pragma once

#include <cstddef>
#include <cstdint>

namespace cansat {

constexpr uint8_t kTelemetryStartByte = 0xA5;
constexpr uint8_t kTelemetryTypeVersion = 0x11;  // type=1, version=1
constexpr std::size_t kTelemetryPacketSize = 50;

struct TelemetryPacket {
  uint16_t sequence;
  uint32_t timestamp_ms;

  int32_t gps_lat_e7;
  int32_t gps_lon_e7;
  int32_t gps_alt_cm;
  int16_t gps_speed_cms;
  int16_t gps_course_cdeg;

  int16_t imu_ax_mg;
  int16_t imu_ay_mg;
  int16_t imu_az_mg;
  int16_t imu_gx_dps10;
  int16_t imu_gy_dps10;
  int16_t imu_gz_dps10;
  int16_t imu_mx_ut10;
  int16_t imu_my_ut10;
  int16_t imu_mz_ut10;

  uint16_t batt_mv;
  int16_t batt_ma;

  uint16_t status_flags;
};

uint16_t crc16_ccitt_false(const uint8_t* data, std::size_t len);
void encode_telemetry_packet(const TelemetryPacket& packet, uint8_t out[kTelemetryPacketSize]);
bool decode_telemetry_packet(const uint8_t* data, std::size_t len, TelemetryPacket& out);

uint32_t sequence_gap(uint16_t previous, uint16_t current);

}  // namespace cansat


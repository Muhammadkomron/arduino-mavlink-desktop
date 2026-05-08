#include "cansat/protocol/telemetry_packet.hpp"

namespace cansat {

namespace {

inline void put_u16_le(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFFU);
  out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

inline void put_u32_le(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFFU);
  out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  out[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  out[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

inline uint16_t get_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

inline uint32_t get_u32_le(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

inline int16_t get_i16_le(const uint8_t* data) {
  return static_cast<int16_t>(get_u16_le(data));
}

inline int32_t get_i32_le(const uint8_t* data) {
  return static_cast<int32_t>(get_u32_le(data));
}

}  // namespace

uint16_t crc16_ccitt_false(const uint8_t* data, std::size_t len) {
  uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

void encode_telemetry_packet(const TelemetryPacket& packet, uint8_t out[kTelemetryPacketSize]) {
  out[0] = kTelemetryStartByte;
  out[1] = kTelemetryTypeVersion;
  put_u16_le(&out[2], packet.sequence);
  put_u32_le(&out[4], packet.timestamp_ms);

  put_u32_le(&out[8], static_cast<uint32_t>(packet.gps_lat_e7));
  put_u32_le(&out[12], static_cast<uint32_t>(packet.gps_lon_e7));
  put_u32_le(&out[16], static_cast<uint32_t>(packet.gps_alt_cm));
  put_u16_le(&out[20], static_cast<uint16_t>(packet.gps_speed_cms));
  put_u16_le(&out[22], static_cast<uint16_t>(packet.gps_course_cdeg));

  put_u16_le(&out[24], static_cast<uint16_t>(packet.imu_ax_mg));
  put_u16_le(&out[26], static_cast<uint16_t>(packet.imu_ay_mg));
  put_u16_le(&out[28], static_cast<uint16_t>(packet.imu_az_mg));
  put_u16_le(&out[30], static_cast<uint16_t>(packet.imu_gx_dps10));
  put_u16_le(&out[32], static_cast<uint16_t>(packet.imu_gy_dps10));
  put_u16_le(&out[34], static_cast<uint16_t>(packet.imu_gz_dps10));
  put_u16_le(&out[36], static_cast<uint16_t>(packet.imu_mx_ut10));
  put_u16_le(&out[38], static_cast<uint16_t>(packet.imu_my_ut10));
  put_u16_le(&out[40], static_cast<uint16_t>(packet.imu_mz_ut10));

  put_u16_le(&out[42], packet.batt_mv);
  put_u16_le(&out[44], static_cast<uint16_t>(packet.batt_ma));
  put_u16_le(&out[46], packet.status_flags);

  const uint16_t crc = crc16_ccitt_false(out, kTelemetryPacketSize - 2U);
  put_u16_le(&out[48], crc);
}

bool decode_telemetry_packet(const uint8_t* data, std::size_t len, TelemetryPacket& out) {
  if (len != kTelemetryPacketSize) {
    return false;
  }
  if (data[0] != kTelemetryStartByte || data[1] != kTelemetryTypeVersion) {
    return false;
  }

  const uint16_t expected_crc = get_u16_le(&data[48]);
  const uint16_t actual_crc = crc16_ccitt_false(data, kTelemetryPacketSize - 2U);
  if (expected_crc != actual_crc) {
    return false;
  }

  out.sequence = get_u16_le(&data[2]);
  out.timestamp_ms = get_u32_le(&data[4]);

  out.gps_lat_e7 = get_i32_le(&data[8]);
  out.gps_lon_e7 = get_i32_le(&data[12]);
  out.gps_alt_cm = get_i32_le(&data[16]);
  out.gps_speed_cms = get_i16_le(&data[20]);
  out.gps_course_cdeg = get_i16_le(&data[22]);

  out.imu_ax_mg = get_i16_le(&data[24]);
  out.imu_ay_mg = get_i16_le(&data[26]);
  out.imu_az_mg = get_i16_le(&data[28]);
  out.imu_gx_dps10 = get_i16_le(&data[30]);
  out.imu_gy_dps10 = get_i16_le(&data[32]);
  out.imu_gz_dps10 = get_i16_le(&data[34]);
  out.imu_mx_ut10 = get_i16_le(&data[36]);
  out.imu_my_ut10 = get_i16_le(&data[38]);
  out.imu_mz_ut10 = get_i16_le(&data[40]);

  out.batt_mv = get_u16_le(&data[42]);
  out.batt_ma = get_i16_le(&data[44]);
  out.status_flags = get_u16_le(&data[46]);
  return true;
}

uint32_t sequence_gap(uint16_t previous, uint16_t current) {
  const uint16_t delta = static_cast<uint16_t>(current - previous);
  if (delta <= 1U) {
    return 0U;
  }
  return static_cast<uint32_t>(delta - 1U);
}

}  // namespace cansat


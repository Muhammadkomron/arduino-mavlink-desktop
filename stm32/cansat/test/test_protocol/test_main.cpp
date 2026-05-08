#include <unity.h>

#include "cansat/protocol/telemetry_packet.hpp"

using cansat::TelemetryPacket;

void test_crc16_reference_vector(void) {
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, cansat::crc16_ccitt_false(data, sizeof(data)));
}

void test_packet_encode_decode_roundtrip(void) {
  TelemetryPacket tx{};
  tx.sequence = 42;
  tx.timestamp_ms = 123456;
  tx.gps_lat_e7 = 412345678;
  tx.gps_lon_e7 = 69234567;
  tx.gps_alt_cm = 12345;
  tx.gps_speed_cms = 678;
  tx.gps_course_cdeg = 2345;
  tx.imu_ax_mg = 100;
  tx.imu_ay_mg = -200;
  tx.imu_az_mg = 990;
  tx.imu_gx_dps10 = 15;
  tx.imu_gy_dps10 = -25;
  tx.imu_gz_dps10 = 35;
  tx.imu_mx_ut10 = 11;
  tx.imu_my_ut10 = 22;
  tx.imu_mz_ut10 = 33;
  tx.batt_mv = 12000;
  tx.batt_ma = -250;
  tx.status_flags = 0x0155;

  uint8_t raw[cansat::kTelemetryPacketSize]{};
  cansat::encode_telemetry_packet(tx, raw);

  TelemetryPacket rx{};
  TEST_ASSERT_TRUE(cansat::decode_telemetry_packet(raw, sizeof(raw), rx));
  TEST_ASSERT_EQUAL_UINT16(tx.sequence, rx.sequence);
  TEST_ASSERT_EQUAL_UINT32(tx.timestamp_ms, rx.timestamp_ms);
  TEST_ASSERT_EQUAL_INT32(tx.gps_lat_e7, rx.gps_lat_e7);
  TEST_ASSERT_EQUAL_INT32(tx.gps_lon_e7, rx.gps_lon_e7);
  TEST_ASSERT_EQUAL_INT32(tx.gps_alt_cm, rx.gps_alt_cm);
  TEST_ASSERT_EQUAL_INT16(tx.imu_ay_mg, rx.imu_ay_mg);
  TEST_ASSERT_EQUAL_UINT16(tx.batt_mv, rx.batt_mv);
  TEST_ASSERT_EQUAL_INT16(tx.batt_ma, rx.batt_ma);
  TEST_ASSERT_EQUAL_UINT16(tx.status_flags, rx.status_flags);
}

void test_packet_crc_rejects_corruption(void) {
  TelemetryPacket tx{};
  uint8_t raw[cansat::kTelemetryPacketSize]{};
  cansat::encode_telemetry_packet(tx, raw);
  raw[10] ^= 0x5A;

  TelemetryPacket rx{};
  TEST_ASSERT_FALSE(cansat::decode_telemetry_packet(raw, sizeof(raw), rx));
}

void test_sequence_gap_wraparound(void) {
  TEST_ASSERT_EQUAL_UINT32(0, cansat::sequence_gap(10, 11));
  TEST_ASSERT_EQUAL_UINT32(2, cansat::sequence_gap(10, 13));
  TEST_ASSERT_EQUAL_UINT32(0, cansat::sequence_gap(65535, 0));
  TEST_ASSERT_EQUAL_UINT32(5, cansat::sequence_gap(65535, 5));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_reference_vector);
  RUN_TEST(test_packet_encode_decode_roundtrip);
  RUN_TEST(test_packet_crc_rejects_corruption);
  RUN_TEST(test_sequence_gap_wraparound);
  return UNITY_END();
}

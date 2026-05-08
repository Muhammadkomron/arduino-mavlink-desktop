#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

namespace cansat {

struct XBeeFrame {
  uint8_t type = 0;
  uint8_t frame_id = 0;
  uint8_t delivery_status = 0xFF;
  uint8_t discovery_status = 0xFF;
  uint8_t options = 0;
  uint64_t source64 = 0;
  const uint8_t* payload = nullptr;
  std::size_t payload_len = 0;
};

class XBeeApi {
 public:
  explicit XBeeApi(HardwareSerial& serial);

  void begin(uint32_t baud);
  bool send_transmit_request(const uint8_t* payload, std::size_t payload_len, uint64_t dest64, uint8_t& frame_id_out);
  bool poll_frame(XBeeFrame& frame_out);

 private:
  enum class ParseState : uint8_t {
    kWaitStart,
    kLenMsb,
    kLenLsb,
    kData,
    kChecksum,
  };

  HardwareSerial* serial_;
  ParseState state_ = ParseState::kWaitStart;
  uint16_t frame_length_ = 0;
  uint16_t frame_index_ = 0;
  uint8_t checksum_sum_ = 0;
  uint8_t next_frame_id_ = 1;
  uint8_t rx_frame_data_[220]{};

  uint8_t allocate_frame_id();
  static uint64_t parse_u64_be(const uint8_t* data);
};

}  // namespace cansat


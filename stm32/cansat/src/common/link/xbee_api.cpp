#include "cansat/link/xbee_api.hpp"

namespace cansat {

XBeeApi::XBeeApi(HardwareSerial& serial) : serial_(&serial) {}

void XBeeApi::begin(uint32_t baud) { serial_->begin(baud); }

uint8_t XBeeApi::allocate_frame_id() {
  if (next_frame_id_ == 0U) {
    next_frame_id_ = 1U;
  }
  const uint8_t id = next_frame_id_;
  ++next_frame_id_;
  return id;
}

uint64_t XBeeApi::parse_u64_be(const uint8_t* data) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8U; ++i) {
    value = (value << 8ULL) | static_cast<uint64_t>(data[i]);
  }
  return value;
}

bool XBeeApi::send_transmit_request(const uint8_t* payload, std::size_t payload_len, uint64_t dest64, uint8_t& frame_id_out) {
  if (payload_len > 200U) {
    return false;
  }

  uint8_t frame_data[220]{};
  std::size_t idx = 0;
  frame_data[idx++] = 0x10;  // Transmit Request

  const uint8_t frame_id = allocate_frame_id();
  frame_data[idx++] = frame_id;
  frame_id_out = frame_id;

  for (int shift = 56; shift >= 0; shift -= 8) {
    frame_data[idx++] = static_cast<uint8_t>((dest64 >> static_cast<uint8_t>(shift)) & 0xFFULL);
  }

  frame_data[idx++] = 0xFF;  // 16-bit destination unknown
  frame_data[idx++] = 0xFE;
  frame_data[idx++] = 0x00;  // broadcast radius
  frame_data[idx++] = 0x00;  // options (ACK enabled for unicast)

  for (std::size_t i = 0; i < payload_len; ++i) {
    frame_data[idx++] = payload[i];
  }

  const uint16_t frame_len = static_cast<uint16_t>(idx);
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < idx; ++i) {
    checksum = static_cast<uint8_t>(checksum + frame_data[i]);
  }
  checksum = static_cast<uint8_t>(0xFFU - checksum);

  serial_->write(0x7EU);
  serial_->write(static_cast<uint8_t>((frame_len >> 8U) & 0xFFU));
  serial_->write(static_cast<uint8_t>(frame_len & 0xFFU));
  serial_->write(frame_data, idx);
  serial_->write(checksum);
  return true;
}

bool XBeeApi::poll_frame(XBeeFrame& frame_out) {
  while (serial_->available() > 0) {
    const int byte_in = serial_->read();
    if (byte_in < 0) {
      return false;
    }
    const uint8_t b = static_cast<uint8_t>(byte_in);

    switch (state_) {
      case ParseState::kWaitStart:
        if (b == 0x7EU) {
          state_ = ParseState::kLenMsb;
          frame_length_ = 0;
          frame_index_ = 0;
          checksum_sum_ = 0;
        }
        break;

      case ParseState::kLenMsb:
        frame_length_ = static_cast<uint16_t>(b) << 8U;
        state_ = ParseState::kLenLsb;
        break;

      case ParseState::kLenLsb:
        frame_length_ |= b;
        if (frame_length_ == 0U || frame_length_ > sizeof(rx_frame_data_)) {
          state_ = ParseState::kWaitStart;
        } else {
          frame_index_ = 0;
          checksum_sum_ = 0;
          state_ = ParseState::kData;
        }
        break;

      case ParseState::kData:
        rx_frame_data_[frame_index_++] = b;
        checksum_sum_ = static_cast<uint8_t>(checksum_sum_ + b);
        if (frame_index_ >= frame_length_) {
          state_ = ParseState::kChecksum;
        }
        break;

      case ParseState::kChecksum: {
        const uint8_t expected = static_cast<uint8_t>(0xFFU - checksum_sum_);
        state_ = ParseState::kWaitStart;
        if (b != expected) {
          break;
        }
        if (frame_length_ < 1U) {
          break;
        }

        frame_out = {};
        frame_out.type = rx_frame_data_[0];

        if (frame_out.type == 0x8BU && frame_length_ >= 7U) {
          frame_out.frame_id = rx_frame_data_[1];
          frame_out.delivery_status = rx_frame_data_[5];
          frame_out.discovery_status = rx_frame_data_[6];
          return true;
        }

        if (frame_out.type == 0x90U && frame_length_ >= 12U) {
          frame_out.source64 = parse_u64_be(&rx_frame_data_[1]);
          frame_out.options = rx_frame_data_[11];
          frame_out.payload = &rx_frame_data_[12];
          frame_out.payload_len = static_cast<std::size_t>(frame_length_ - 12U);
          return true;
        }

        return true;
      }
    }
  }

  return false;
}

}  // namespace cansat


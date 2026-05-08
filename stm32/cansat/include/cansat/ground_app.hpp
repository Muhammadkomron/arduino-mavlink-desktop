#pragma once

#include <Arduino.h>

#include "cansat/board_config.hpp"
#include "cansat/link/xbee_api.hpp"

namespace cansat {

class GroundApp {
 public:
  GroundApp();
  void setup();
  void loop();

 private:
  HardwareSerial xbee_serial_;
  XBeeApi xbee_;
  bool have_last_seq_ = false;
  uint16_t last_seq_ = 0;
  uint32_t lost_packets_ = 0;
  uint32_t crc_errors_ = 0;

  void process_frame(const XBeeFrame& frame);
};

}  // namespace cansat


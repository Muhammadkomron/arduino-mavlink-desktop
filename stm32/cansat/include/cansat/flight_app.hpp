#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "cansat/board_config.hpp"
#include "cansat/link/xbee_api.hpp"
#include "cansat/sensors/gps_nmea.hpp"
#include "cansat/sensors/imu_mpu9250.hpp"
#include "cansat/sensors/power_ina219.hpp"

namespace cansat {

class FlightApp {
 public:
  FlightApp();
  void setup();
  void loop();

 private:
  HardwareSerial xbee_serial_;
  HardwareSerial gps_serial_;
  TwoWire imu_bus_;
  TwoWire ina_bus_;

  XBeeApi xbee_;
  ImuMpu9250 imu_;
  PowerIna219 ina_;
  GpsNmea gps_;

  uint16_t sequence_ = 0;
  uint32_t last_send_ms_ = 0;
  uint8_t pending_frame_id_ = 0;
  bool last_delivery_ok_ = false;
  bool imu_ready_ = false;
  bool ina_ready_ = false;

  void process_xbee_status();
  void send_telemetry_tick();
  static void blink_once(uint32_t pin, uint16_t ms = 30);
};

}  // namespace cansat


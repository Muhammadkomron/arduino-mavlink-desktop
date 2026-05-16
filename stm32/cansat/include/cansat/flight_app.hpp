#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "cansat/actuators/servo_pair.hpp"
#include "cansat/board_config.hpp"
#include "cansat/data_models.hpp"
#include "cansat/guidance/guidance.hpp"
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

  ServoPair servos_;
  Guidance  guidance_;

  ImuData last_imu_{};
  PowerData last_power_{};

  uint16_t sequence_ = 0;
  uint32_t last_send_ms_     = 0;
  uint32_t last_guidance_ms_ = 0;
  uint32_t last_imu_poll_ms_ = 0;
  uint32_t last_ina_poll_ms_ = 0;

  uint8_t pending_frame_id_ = 0;
  bool last_delivery_ok_    = false;
  bool imu_ready_           = false;
  bool ina_ready_           = false;
  bool last_imu_valid_      = false;
  bool last_power_valid_    = false;
  uint8_t last_guidance_mode_ = 0;

  void poll_imu();
  void poll_ina();
  void process_xbee_status();
  void guidance_tick();
  void send_telemetry_tick();
  void run_servo_selftest();
  static void blink_once(uint32_t pin, uint16_t ms = 30);
};

}  // namespace cansat

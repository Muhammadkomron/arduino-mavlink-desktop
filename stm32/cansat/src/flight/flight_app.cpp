#include "cansat/flight_app.hpp"

#include "cansat/data_models.hpp"
#include "cansat/protocol/telemetry_packet.hpp"
#include "cansat/status_flags.hpp"

namespace cansat {

FlightApp::FlightApp()
    : xbee_serial_(config::kXBeeRxPin, config::kXBeeTxPin),
      gps_serial_(config::kGpsRxPin, config::kGpsTxPin),
      imu_bus_(config::kImuSdaPin, config::kImuSclPin),
      ina_bus_(config::kInaSdaPin, config::kInaSclPin),
      xbee_(xbee_serial_) {}

void FlightApp::blink_once(uint32_t pin, uint16_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

void FlightApp::setup() {
  pinMode(config::kLedOk, OUTPUT);
  pinMode(config::kLedTx, OUTPUT);
  pinMode(config::kLedErr, OUTPUT);

  blink_once(config::kLedOk, 120);
  blink_once(config::kLedTx, 120);
  blink_once(config::kLedErr, 120);

  xbee_.begin(config::kXBeeBaud);
  gps_.begin(gps_serial_, config::kGpsBaud);

  imu_bus_.begin();
  ina_bus_.begin();
  imu_ready_ = imu_.begin(imu_bus_);
  ina_ready_ = ina_.begin(ina_bus_);
}

void FlightApp::process_xbee_status() {
  XBeeFrame frame{};
  while (xbee_.poll_frame(frame)) {
    if (frame.type == 0x8B && frame.frame_id == pending_frame_id_) {
      last_delivery_ok_ = (frame.delivery_status == 0x00);
      pending_frame_id_ = 0;
      if (last_delivery_ok_) {
        blink_once(config::kLedOk, 20);
      } else {
        blink_once(config::kLedErr, 25);
      }
    }
  }
}

void FlightApp::send_telemetry_tick() {
  TelemetryPacket packet{};
  packet.sequence = sequence_++;
  packet.timestamp_ms = millis();
  packet.status_flags = STATUS_BME280_DISABLED;

  ImuData imu_data{};
  if (imu_ready_ && imu_.read(imu_data)) {
    packet.imu_ax_mg = imu_data.ax_mg;
    packet.imu_ay_mg = imu_data.ay_mg;
    packet.imu_az_mg = imu_data.az_mg;
    packet.imu_gx_dps10 = imu_data.gx_dps10;
    packet.imu_gy_dps10 = imu_data.gy_dps10;
    packet.imu_gz_dps10 = imu_data.gz_dps10;
    packet.imu_mx_ut10 = imu_data.mx_ut10;
    packet.imu_my_ut10 = imu_data.my_ut10;
    packet.imu_mz_ut10 = imu_data.mz_ut10;
    packet.status_flags |= STATUS_IMU_OK | STATUS_IMU_FRESH;
    if (imu_data.mag_ok) {
      packet.status_flags |= STATUS_MAG_OK;
    }
  }

  PowerData power_data{};
  if (ina_ready_ && ina_.read(power_data)) {
    packet.batt_mv = power_data.bus_mv;
    packet.batt_ma = power_data.current_ma;
    packet.status_flags |= STATUS_INA_OK | STATUS_INA_FRESH;
  }

  const GpsData gps_data = gps_.latest();
  packet.gps_lat_e7 = gps_data.lat_e7;
  packet.gps_lon_e7 = gps_data.lon_e7;
  packet.gps_alt_cm = gps_data.alt_cm;
  packet.gps_speed_cms = gps_data.speed_cms;
  packet.gps_course_cdeg = gps_data.course_cdeg;
  if (gps_data.fix) {
    packet.status_flags |= STATUS_GPS_FIX;
  }
  if (gps_data.fresh) {
    packet.status_flags |= STATUS_GPS_FRESH;
  }
  if (last_delivery_ok_) {
    packet.status_flags |= STATUS_XBEE_DELIVERY_OK;
  }

  uint8_t payload[kTelemetryPacketSize]{};
  encode_telemetry_packet(packet, payload);

  uint8_t frame_id = 0;
  const bool sent = xbee_.send_transmit_request(payload, sizeof(payload), config::kXBeeDestination64, frame_id);
  blink_once(config::kLedTx);
  if (sent) {
    pending_frame_id_ = frame_id;
  } else {
    blink_once(config::kLedErr, 40);
  }
}

void FlightApp::loop() {
  gps_.poll();
  process_xbee_status();

  const uint32_t now = millis();
  if (now - last_send_ms_ >= config::kTelemetryPeriodMs) {
    last_send_ms_ = now;
    send_telemetry_tick();
  }
}

}  // namespace cansat

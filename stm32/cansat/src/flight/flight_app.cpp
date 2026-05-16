#include "cansat/flight_app.hpp"

#include "cansat/data_models.hpp"
#include "cansat/protocol/telemetry_packet.hpp"
#include "cansat/status_flags.hpp"

namespace cansat {

namespace {
constexpr uint32_t kImuPollPeriodMs = 100;  // 10 Hz IMU snapshot
constexpr uint32_t kInaPollPeriodMs = 250;  // 4 Hz battery snapshot
}  // namespace

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

  servos_.begin();
  servos_.neutral();
  run_servo_selftest();
  guidance_.begin();
}

void FlightApp::run_servo_selftest() {
  // ~10 s on power-up: ramps each servo from rest to the configured maximum
  // angle (`ServoPair::kMaxAngleDeg`) and back, one side at a time, so you can
  // confirm both channels are alive, mapped to the correct side, and pulling
  // in the expected direction. The driver enforces the angle cap internally -
  // an upstream bug commanding 150 deg would still produce only `kMaxAngleDeg`
  // of travel. Delete or guard with an #ifdef once you don't want servos
  // thrashing at every boot.
  constexpr uint8_t  kSteps  = 50;
  constexpr uint16_t kStepMs = 50;  // 50 steps * 50 ms = 2.5 s per ramp leg

  const float max_deg = static_cast<float>(ServoPair::kMaxAngleDeg);

  for (uint8_t i = 0; i <= kSteps; ++i) {
    const float a = max_deg * static_cast<float>(i) / static_cast<float>(kSteps);
    servos_.set_left_angle_deg(a);
    servos_.set_right_angle_deg(0.0f);
    delay(kStepMs);
  }
  for (uint8_t i = kSteps; i > 0; --i) {
    const float a = max_deg * static_cast<float>(i - 1) / static_cast<float>(kSteps);
    servos_.set_left_angle_deg(a);
    servos_.set_right_angle_deg(0.0f);
    delay(kStepMs);
  }
  for (uint8_t i = 0; i <= kSteps; ++i) {
    const float a = max_deg * static_cast<float>(i) / static_cast<float>(kSteps);
    servos_.set_left_angle_deg(0.0f);
    servos_.set_right_angle_deg(a);
    delay(kStepMs);
  }
  for (uint8_t i = kSteps; i > 0; --i) {
    const float a = max_deg * static_cast<float>(i - 1) / static_cast<float>(kSteps);
    servos_.set_left_angle_deg(0.0f);
    servos_.set_right_angle_deg(a);
    delay(kStepMs);
  }
  servos_.neutral();
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

void FlightApp::poll_imu() {
  if (!imu_ready_) {
    return;
  }
  ImuData fresh{};
  if (imu_.read(fresh)) {
    last_imu_ = fresh;
    last_imu_valid_ = true;
  }
}

void FlightApp::poll_ina() {
  if (!ina_ready_) {
    return;
  }
  PowerData fresh{};
  if (ina_.read(fresh)) {
    last_power_ = fresh;
    last_power_valid_ = true;
  }
}

void FlightApp::guidance_tick() {
  const GpsData gps_data = gps_.latest();

  GuidanceInput in{};
  in.gps_lat_e7      = gps_data.lat_e7;
  in.gps_lon_e7      = gps_data.lon_e7;
  in.gps_alt_cm      = gps_data.alt_cm;
  in.gps_speed_cms   = gps_data.speed_cms;
  in.gps_course_cdeg = gps_data.course_cdeg;
  in.gps_fix         = gps_data.fix;

  if (last_imu_valid_) {
    in.gz_dps10 = last_imu_.gz_dps10;
    in.mx_ut10  = last_imu_.mx_ut10;
    in.my_ut10  = last_imu_.my_ut10;
    in.mz_ut10  = last_imu_.mz_ut10;
    in.mag_ok   = last_imu_.mag_ok;
  }

  const GuidanceOutput out = guidance_.tick(in, millis());
  servos_.set_pair(out.left_pull, out.right_pull);
  last_guidance_mode_ = out.mode;
}

void FlightApp::send_telemetry_tick() {
  TelemetryPacket packet{};
  packet.sequence     = sequence_++;
  packet.timestamp_ms = millis();
  packet.status_flags = STATUS_BME280_DISABLED;

  if (last_imu_valid_) {
    packet.imu_ax_mg    = last_imu_.ax_mg;
    packet.imu_ay_mg    = last_imu_.ay_mg;
    packet.imu_az_mg    = last_imu_.az_mg;
    packet.imu_gx_dps10 = last_imu_.gx_dps10;
    packet.imu_gy_dps10 = last_imu_.gy_dps10;
    packet.imu_gz_dps10 = last_imu_.gz_dps10;
    packet.imu_mx_ut10  = last_imu_.mx_ut10;
    packet.imu_my_ut10  = last_imu_.my_ut10;
    packet.imu_mz_ut10  = last_imu_.mz_ut10;
    packet.status_flags |= STATUS_IMU_OK | STATUS_IMU_FRESH;
    if (last_imu_.mag_ok) {
      packet.status_flags |= STATUS_MAG_OK;
    }
  }

  if (last_power_valid_) {
    packet.batt_mv = last_power_.bus_mv;
    packet.batt_ma = last_power_.current_ma;
    packet.status_flags |= STATUS_INA_OK | STATUS_INA_FRESH;
  }

  const GpsData gps_data = gps_.latest();
  packet.gps_lat_e7      = gps_data.lat_e7;
  packet.gps_lon_e7      = gps_data.lon_e7;
  packet.gps_alt_cm      = gps_data.alt_cm;
  packet.gps_speed_cms   = gps_data.speed_cms;
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
  if (now - last_imu_poll_ms_ >= kImuPollPeriodMs) {
    last_imu_poll_ms_ = now;
    poll_imu();
  }
  if (now - last_ina_poll_ms_ >= kInaPollPeriodMs) {
    last_ina_poll_ms_ = now;
    poll_ina();
  }
  if (now - last_guidance_ms_ >= guidance_config::kGuidancePeriodMs) {
    last_guidance_ms_ = now;
    guidance_tick();
  }
  if (now - last_send_ms_ >= config::kTelemetryPeriodMs) {
    last_send_ms_ = now;
    send_telemetry_tick();
  }
}

}  // namespace cansat

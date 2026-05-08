#include "cansat/ground_app.hpp"

#include "cansat/protocol/telemetry_packet.hpp"
#include "cansat/status_flags.hpp"

namespace cansat {

GroundApp::GroundApp() : xbee_serial_(config::kXBeeRxPin, config::kXBeeTxPin), xbee_(xbee_serial_) {}

void GroundApp::setup() {
  Serial.begin(config::kDebugBaud);

  pinMode(config::kLedOk, OUTPUT);
  pinMode(config::kLedErr, OUTPUT);
  pinMode(config::kLedTx, OUTPUT);

  xbee_.begin(config::kXBeeBaud);

  Serial.println(F("=== GROUND TELEMETRY RECEIVER ==="));
  Serial.println(F("CSV: seq,lost,total_lost,time_ms,lat_e7,lon_e7,alt_cm,vbat_mv,ibat_ma,flags"));
}

void GroundApp::process_frame(const XBeeFrame& frame) {
  if (frame.type != 0x90) {
    return;
  }

  TelemetryPacket packet{};
  if (!decode_telemetry_packet(frame.payload, frame.payload_len, packet)) {
    ++crc_errors_;
    digitalWrite(config::kLedErr, HIGH);
    delay(5);
    digitalWrite(config::kLedErr, LOW);
    return;
  }

  uint32_t gap = 0;
  if (have_last_seq_) {
    gap = sequence_gap(last_seq_, packet.sequence);
    lost_packets_ += gap;
  }
  last_seq_ = packet.sequence;
  have_last_seq_ = true;

  digitalWrite(config::kLedOk, HIGH);
  delay(3);
  digitalWrite(config::kLedOk, LOW);

  Serial.print(packet.sequence);
  Serial.print(',');
  Serial.print(gap);
  Serial.print(',');
  Serial.print(lost_packets_);
  Serial.print(',');
  Serial.print(packet.timestamp_ms);
  Serial.print(',');
  Serial.print(packet.gps_lat_e7);
  Serial.print(',');
  Serial.print(packet.gps_lon_e7);
  Serial.print(',');
  Serial.print(packet.gps_alt_cm);
  Serial.print(',');
  Serial.print(packet.batt_mv);
  Serial.print(',');
  Serial.print(packet.batt_ma);
  Serial.print(',');
  Serial.print(packet.status_flags, HEX);
  Serial.print(',');
  Serial.println(crc_errors_);
}

void GroundApp::loop() {
  XBeeFrame frame{};
  while (xbee_.poll_frame(frame)) {
    process_frame(frame);
  }
}

}  // namespace cansat

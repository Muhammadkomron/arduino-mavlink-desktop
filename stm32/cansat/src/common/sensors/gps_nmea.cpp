#include "cansat/sensors/gps_nmea.hpp"

#include <cstdlib>
#include <cstring>

namespace cansat {

namespace {

char* next_field(char*& cursor) {
  if (cursor == nullptr) {
    return nullptr;
  }
  char* start = cursor;
  while (*cursor != '\0' && *cursor != ',') {
    ++cursor;
  }
  if (*cursor == ',') {
    *cursor = '\0';
    ++cursor;
  } else {
    cursor = nullptr;
  }
  return start;
}

}  // namespace

bool GpsNmea::begin(HardwareSerial& serial, uint32_t baud) {
  serial_ = &serial;
  serial_->begin(baud);
  line_len_ = 0;
  latest_ = {};
  last_update_ms_ = 0;
  return true;
}

void GpsNmea::poll() {
  if (serial_ == nullptr) {
    return;
  }

  latest_.fresh = false;

  while (serial_->available() > 0) {
    const int value = serial_->read();
    if (value < 0) {
      break;
    }

    const char c = static_cast<char>(value);
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (line_len_ > 0) {
        line_buf_[line_len_] = '\0';
        parse_line(line_buf_);
        line_len_ = 0;
      }
      continue;
    }

    if (line_len_ < static_cast<uint8_t>(sizeof(line_buf_) - 1U)) {
      line_buf_[line_len_++] = c;
    } else {
      line_len_ = 0;
    }
  }
}

GpsData GpsNmea::latest() const { return latest_; }

void GpsNmea::parse_line(char* line) {
  if (line == nullptr || line[0] != '$') {
    return;
  }

  if (strstr(line, "GGA") != nullptr) {
    parse_gga(line);
  } else if (strstr(line, "RMC") != nullptr) {
    parse_rmc(line);
  }
}

void GpsNmea::parse_gga(char* line) {
  char* cursor = line;
  (void)next_field(cursor);              // $xxGGA
  (void)next_field(cursor);              // utc
  const char* lat = next_field(cursor);  // latitude
  const char* ns = next_field(cursor);   // N/S
  const char* lon = next_field(cursor);  // longitude
  const char* ew = next_field(cursor);   // E/W
  const char* quality = next_field(cursor);
  const char* sats = next_field(cursor);
  (void)next_field(cursor);               // hdop
  const char* alt = next_field(cursor);   // altitude m

  if (lat != nullptr && ns != nullptr && lon != nullptr && ew != nullptr) {
    latest_.lat_e7 = parse_coord_e7(lat, ns[0]);
    latest_.lon_e7 = parse_coord_e7(lon, ew[0]);
  }

  if (quality != nullptr) {
    latest_.fix = (quality[0] >= '1' && quality[0] <= '9');
  }
  if (alt != nullptr && alt[0] != '\0') {
    latest_.alt_cm = parse_decimal_scaled(alt, 100);
  }
  (void)sats;

  latest_.fresh = true;
  last_update_ms_ = millis();
}

void GpsNmea::parse_rmc(char* line) {
  char* cursor = line;
  (void)next_field(cursor);                  // $xxRMC
  (void)next_field(cursor);                  // utc
  const char* status = next_field(cursor);   // A/V
  const char* lat = next_field(cursor);      // latitude
  const char* ns = next_field(cursor);       // N/S
  const char* lon = next_field(cursor);      // longitude
  const char* ew = next_field(cursor);       // E/W
  const char* speed = next_field(cursor);    // knots
  const char* course = next_field(cursor);   // degrees

  if (status != nullptr) {
    latest_.fix = (status[0] == 'A');
  }
  if (lat != nullptr && ns != nullptr && lon != nullptr && ew != nullptr) {
    latest_.lat_e7 = parse_coord_e7(lat, ns[0]);
    latest_.lon_e7 = parse_coord_e7(lon, ew[0]);
  }
  if (speed != nullptr && speed[0] != '\0') {
    const int32_t speed_knots_x100 = parse_decimal_scaled(speed, 100);
    latest_.speed_cms = static_cast<int16_t>((speed_knots_x100 * 5144L) / 10000L);
  }
  if (course != nullptr && course[0] != '\0') {
    latest_.course_cdeg = static_cast<int16_t>(parse_decimal_scaled(course, 100));
  }

  latest_.fresh = true;
  last_update_ms_ = millis();
}

int32_t GpsNmea::parse_coord_e7(const char* value, char hemi) {
  if (value == nullptr || value[0] == '\0') {
    return 0;
  }
  const double raw = std::strtod(value, nullptr);
  const int32_t degrees = static_cast<int32_t>(raw / 100.0);
  const double minutes = raw - (static_cast<double>(degrees) * 100.0);
  double decimal = static_cast<double>(degrees) + (minutes / 60.0);
  if (hemi == 'S' || hemi == 'W') {
    decimal = -decimal;
  }
  return static_cast<int32_t>(decimal * 10000000.0);
}

int32_t GpsNmea::parse_decimal_scaled(const char* value, int32_t scale) {
  if (value == nullptr || value[0] == '\0') {
    return 0;
  }
  const double parsed = std::strtod(value, nullptr);
  return static_cast<int32_t>(parsed * static_cast<double>(scale));
}

}  // namespace cansat


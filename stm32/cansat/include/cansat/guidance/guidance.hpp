#pragma once

#include <cstdint>

#include "cansat/guidance_config.hpp"

namespace cansat {

// Sensor inputs the guidance loop reads each tick. Units match the rest of the
// project (GpsData scaled-int convention plus a bool for the magnetometer).
struct GuidanceInput {
  int32_t gps_lat_e7      = 0;
  int32_t gps_lon_e7      = 0;
  int32_t gps_alt_cm      = 0;
  int16_t gps_speed_cms   = 0;
  int16_t gps_course_cdeg = 0;
  bool    gps_fix         = false;

  int16_t gz_dps10        = 0;  // body-frame yaw rate, deg/s * 10
  int16_t mx_ut10         = 0;
  int16_t my_ut10         = 0;
  int16_t mz_ut10         = 0;
  bool    mag_ok          = false;
};

// Commands and diagnostics produced each tick.
struct GuidanceOutput {
  float    left_pull   = 0.0f;  // [0,1]
  float    right_pull  = 0.0f;  // [0,1]
  float    heading_rad = 0.0f;  // current best heading estimate, [-pi,pi]
  float    bearing_rad = 0.0f;  // bearing from craft to target, [-pi,pi]
  float    distance_m  = 0.0f;  // horizontal distance to target
  float    alt_agl_m   = 0.0f;  // altitude above target ground
  uint8_t  mode        = 0;     // see GuidanceMode
};

enum GuidanceMode : uint8_t {
  GUIDANCE_STANDBY = 0,  // no fix, or above kAltStartGuidanceM
  GUIDANCE_CRUISE  = 1,  // steering toward target, well above the ground
  GUIDANCE_FINAL   = 2,  // near the ground, locked on bearing
  GUIDANCE_FLARE   = 3,  // both servos pulled, slowing for touchdown
  GUIDANCE_LANDED  = 4,  // assumed on the ground, servos released
};

class Guidance {
 public:
  void begin();

  // Run one control step. Returns the desired servo pulls plus diagnostics.
  GuidanceOutput tick(const GuidanceInput& in, uint32_t now_ms);

  GuidanceMode mode() const { return mode_; }

 private:
  GuidanceMode mode_     = GUIDANCE_STANDBY;
  uint32_t last_tick_ms_ = 0;

  static float wrap_pi(float rad);
  static void  bearing_and_distance(int32_t lat_e7, int32_t lon_e7,
                                    int32_t tgt_lat_e7, int32_t tgt_lon_e7,
                                    float& bearing_rad_out,
                                    float& distance_m_out);
  static float estimate_heading_rad(const GuidanceInput& in);
};

}  // namespace cansat

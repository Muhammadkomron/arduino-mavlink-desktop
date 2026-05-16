#include "cansat/guidance/guidance.hpp"

#include <math.h>

namespace cansat {

namespace {

constexpr float kPi             = 3.14159265358979323846f;
constexpr float kTwoPi          = 2.0f * kPi;
constexpr float kDegToRad       = kPi / 180.0f;
constexpr float kMetersPerLatDeg = 111320.0f;  // good to ~0.5% globally

}  // namespace

void Guidance::begin() {
  mode_ = GUIDANCE_STANDBY;
  last_tick_ms_ = 0;
}

float Guidance::wrap_pi(float rad) {
  while (rad >  kPi) rad -= kTwoPi;
  while (rad < -kPi) rad += kTwoPi;
  return rad;
}

void Guidance::bearing_and_distance(int32_t lat_e7, int32_t lon_e7,
                                    int32_t tgt_lat_e7, int32_t tgt_lon_e7,
                                    float& bearing_rad_out,
                                    float& distance_m_out) {
  // Flat-earth approximation: well within 0.1% error for sub-km distances.
  const float lat_rad  = static_cast<float>(lat_e7) * 1.0e-7f * kDegToRad;
  const float dlat_deg = static_cast<float>(tgt_lat_e7 - lat_e7) * 1.0e-7f;
  const float dlon_deg = static_cast<float>(tgt_lon_e7 - lon_e7) * 1.0e-7f;

  const float dy = dlat_deg * kMetersPerLatDeg;                       // north (+) / south (-)
  const float dx = dlon_deg * kMetersPerLatDeg * cosf(lat_rad);       // east  (+) / west  (-)

  bearing_rad_out = atan2f(dx, dy);            // CW from north, [-pi,pi]
  distance_m_out  = sqrtf(dx * dx + dy * dy);
}

float Guidance::estimate_heading_rad(const GuidanceInput& in) {
  const float speed_mps = static_cast<float>(in.gps_speed_cms) * 0.01f;

  // While moving, GPS course-over-ground is the most reliable heading we have.
  if (in.gps_fix && speed_mps > guidance_config::kMinSpeedForGpsCourseMps) {
    return wrap_pi(static_cast<float>(in.gps_course_cdeg) * 0.01f * kDegToRad);
  }

  // Fallback: 2D magnetometer heading. Assumes the payload is near-level; if
  // it isn't, tilt-compensation (using accel) should be added here.
  if (in.mag_ok) {
    const float mx = static_cast<float>(in.mx_ut10 - guidance_config::kMagOffsetX_uT10);
    const float my = static_cast<float>(in.my_ut10 - guidance_config::kMagOffsetY_uT10);
    float h = atan2f(my, mx);
    h += static_cast<float>(guidance_config::kImuYawOffsetCdeg) * 0.01f * kDegToRad;
    h += static_cast<float>(guidance_config::kMagDeclinationCdeg) * 0.01f * kDegToRad;
    return wrap_pi(h);
  }

  return 0.0f;  // controller will see error vs zero and back off
}

GuidanceOutput Guidance::tick(const GuidanceInput& in, uint32_t now_ms) {
  GuidanceOutput out{};
  out.alt_agl_m  = static_cast<float>(in.gps_alt_cm) * 0.01f - guidance_config::kTargetGroundAltM;
  out.heading_rad = estimate_heading_rad(in);

  bearing_and_distance(in.gps_lat_e7, in.gps_lon_e7,
                       guidance_config::kTargetLatE7,
                       guidance_config::kTargetLonE7,
                       out.bearing_rad, out.distance_m);

  // Phase selection. Without a GPS fix we never command anything.
  if (!in.gps_fix) {
    mode_ = GUIDANCE_STANDBY;
  } else if (out.alt_agl_m <= guidance_config::kAltLandedM) {
    mode_ = GUIDANCE_LANDED;
  } else if (out.alt_agl_m <= guidance_config::kAltFlareM) {
    mode_ = GUIDANCE_FLARE;
  } else if (out.alt_agl_m <= guidance_config::kAltFinalM) {
    mode_ = GUIDANCE_FINAL;
  } else if (out.alt_agl_m <= guidance_config::kAltStartGuidanceM) {
    mode_ = GUIDANCE_CRUISE;
  } else {
    mode_ = GUIDANCE_STANDBY;
  }
  out.mode = static_cast<uint8_t>(mode_);

  switch (mode_) {
    case GUIDANCE_STANDBY:
    case GUIDANCE_LANDED:
      out.left_pull  = 0.0f;
      out.right_pull = 0.0f;
      last_tick_ms_ = now_ms;
      return out;

    case GUIDANCE_FLARE:
      out.left_pull  = guidance_config::kFlareAmount;
      out.right_pull = guidance_config::kFlareAmount;
      last_tick_ms_ = now_ms;
      return out;

    case GUIDANCE_CRUISE:
    case GUIDANCE_FINAL:
    default:
      break;
  }

  // CRUISE / FINAL share the same bearing-tracker for the first cut. Add a
  // wind-aware IP approach later by replacing the bearing target during CRUISE.
  const float yaw_error    = wrap_pi(out.bearing_rad - out.heading_rad);
  const float yaw_rate_rps = static_cast<float>(in.gz_dps10) * 0.1f * kDegToRad;

  float delta = guidance_config::kKpHeading * yaw_error
              - guidance_config::kKdYawRate * yaw_rate_rps;

  const float lim = guidance_config::kBrakeMaxFraction;
  if (delta >  lim) { delta =  lim; }
  if (delta < -lim) { delta = -lim; }

  if (delta >= 0.0f) {
    out.right_pull = delta;
    out.left_pull  = 0.0f;
  } else {
    out.right_pull = 0.0f;
    out.left_pull  = -delta;
  }

  last_tick_ms_ = now_ms;
  return out;
}

}  // namespace cansat

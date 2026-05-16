#pragma once

#include <cstdint>

namespace cansat::guidance_config {

// ---------------------------------------------------------------------------
// Target landing point
// ---------------------------------------------------------------------------
// Set before each flight. Lat/lon are stored as int32 scaled by 1e7 — same
// convention as GpsData::lat_e7 / lon_e7. Override via -D in platformio.ini if
// you prefer per-build configuration.
#ifndef GUIDANCE_TARGET_LAT_E7
#define GUIDANCE_TARGET_LAT_E7 (413111000L)   // 41.3111000 N (example: Tashkent)
#endif
#ifndef GUIDANCE_TARGET_LON_E7
#define GUIDANCE_TARGET_LON_E7 (692797000L)   // 69.2797000 E
#endif
#ifndef GUIDANCE_TARGET_GROUND_ALT_M
#define GUIDANCE_TARGET_GROUND_ALT_M (450)    // m MSL at the target ground level
#endif

constexpr int32_t kTargetLatE7 = GUIDANCE_TARGET_LAT_E7;
constexpr int32_t kTargetLonE7 = GUIDANCE_TARGET_LON_E7;
constexpr float   kTargetGroundAltM = static_cast<float>(GUIDANCE_TARGET_GROUND_ALT_M);

// ---------------------------------------------------------------------------
// MPU9250 mounting offsets
// ---------------------------------------------------------------------------
// The IMU board is not necessarily aligned with the payload's "forward" axis
// (the imaginary line through the two servo wheels). These offsets rotate the
// IMU readings into body frame before the controller consumes them.
//
// All angles are in centidegrees (deg * 100), signed.
//
//   Yaw   - rotation about body Z (vertical when payload hangs level).
//           Positive = IMU yawed clockwise (viewed from above) relative to
//           body-forward. The IMU's reported heading is corrected by
//           subtracting this value.
//   Pitch - rotation about body Y (right-wing axis).
//   Roll  - rotation about body X (forward axis).
//
// For small offsets (< ~15 deg) the gyro yaw rate signal is used as-is; only
// the magnetometer heading is corrected. Tilt-compensation using pitch/roll
// is left as a future improvement once the basic loop is flying.
constexpr int32_t kImuYawOffsetCdeg   = 0;
constexpr int32_t kImuPitchOffsetCdeg = 0;
constexpr int32_t kImuRollOffsetCdeg  = 0;

// Magnetic declination at the launch site (deg * 100). True heading equals
// magnetic heading plus declination. Positive east, negative west. Look up
// for the launch location at https://www.ngdc.noaa.gov/geomag/calculators.
constexpr int32_t kMagDeclinationCdeg = 0;

// Hard-iron offsets (uT*10, same units as ImuData::mx_ut10). Subtracted from
// each magnetometer axis before computing heading. Calibrate by rotating the
// payload through all orientations and recording per-axis min/max — the
// offset is (max+min)/2 on each axis.
constexpr int16_t kMagOffsetX_uT10 = 0;
constexpr int16_t kMagOffsetY_uT10 = 0;
constexpr int16_t kMagOffsetZ_uT10 = 0;

// ---------------------------------------------------------------------------
// Flight-phase altitude thresholds (m AGL, where AGL = gps_alt - kTargetGroundAlt)
// ---------------------------------------------------------------------------
constexpr float kAltStartGuidanceM = 250.0f;   // begin steering at/below this
constexpr float kAltFinalM         =  80.0f;   // switch to direct final mode
constexpr float kAltFlareM         =   8.0f;   // begin symmetric flare
constexpr float kAltLandedM        =   1.0f;   // freeze servos, mission complete

// ---------------------------------------------------------------------------
// Control gains
// ---------------------------------------------------------------------------
constexpr float kKpHeading        = 0.6f;  // brake fraction per rad of yaw error
constexpr float kKdYawRate        = 0.3f;  // brake fraction per (rad/s) of yaw rate
constexpr float kBrakeMaxFraction = 0.8f;  // never command past this fraction (avoid stall)
constexpr float kFlareAmount      = 1.0f;  // both servos pulled this much during flare

// Minimum ground speed (m/s) below which GPS course is unreliable; fall back
// to magnetometer heading.
constexpr float kMinSpeedForGpsCourseMps = 2.0f;

// Guidance loop period.
constexpr uint32_t kGuidancePeriodMs = 200;  // 5 Hz

}  // namespace cansat::guidance_config

#pragma once

#include <cstdint>

class HardwareTimer;

namespace cansat {

// Drives the two paraglider steering servos as a hardware-PWM pair on TIM1.
//
// Pin map (locked by hardware wiring of the airframe):
//   PE11 -> TIM1_CH2 -> RIGHT servo
//   PE13 -> TIM1_CH3 -> LEFT  servo
//
// Standard hobby-servo timing: 50 Hz frame, 1.5 ms = rest, longer pulse =
// further rotation. Most servos map 500 us of pulse change to 90 deg of
// mechanical travel (i.e. the classic 1.0-2.0 ms span covers 180 deg total).
//
// Angle protection
// ----------------
// `kMaxAngleDeg` defines the maximum commanded rotation from rest. The PWM
// output is then bounded so the servo CANNOT be commanded past this limit,
// regardless of what the upstream controller asks for. `pull = 1.0` in the
// API corresponds to exactly `kMaxAngleDeg` of travel - not 180 deg of
// mechanical range. This is a defensive cap: if the guidance computes a
// command that would correspond to e.g. 150 deg of rotation, the API will
// silently clamp it to `kMaxAngleDeg`.
//
// Override at compile time with `-D SERVO_MAX_ANGLE_DEG=60` (etc.) if your
// brake-line geometry needs a different limit.
class ServoPair {
 public:
  static constexpr uint16_t kPeriodUs = 20000;  // 50 Hz frame
  static constexpr uint16_t kRestUs   = 1500;   // line slack, canopy neutral

  // Hardware constant: standard hobby servos move ~5.56 us per degree.
  // Stored as us/deg * 100 so the math stays in integer constexpr.
  static constexpr uint16_t kUsPerDegX100 = 556;  // 500 us / 90 deg

#ifndef SERVO_MAX_ANGLE_DEG
#define SERVO_MAX_ANGLE_DEG 90
#endif
  static constexpr uint16_t kMaxAngleDeg = SERVO_MAX_ANGLE_DEG;

  static_assert(kMaxAngleDeg > 0 && kMaxAngleDeg <= 180,
                "SERVO_MAX_ANGLE_DEG must be in (0, 180]");

  // Pulse width corresponding to pull = 1.0. Cannot exceed this regardless of
  // upstream command. Computed entirely at compile time.
  static constexpr uint16_t kFullPullUs =
      kRestUs + static_cast<uint16_t>((kMaxAngleDeg * kUsPerDegX100) / 100U);

  void begin();

  // pull is clamped to [0, 1]. 0 = neutral, 1 = kMaxAngleDeg of rotation.
  void set_left(float pull);
  void set_right(float pull);
  void set_pair(float left_pull, float right_pull);

  // Explicit angle interface. Out-of-range values are clamped to
  // [0, kMaxAngleDeg]. Useful for self-test and bench tuning.
  void set_left_angle_deg(float angle_deg);
  void set_right_angle_deg(float angle_deg);

  void neutral();

  bool ready() const { return ready_; }

 private:
  HardwareTimer* timer_ = nullptr;
  bool ready_ = false;

  static uint16_t pull_to_us(float pull);
  static uint16_t angle_deg_to_us(float angle_deg);
};

}  // namespace cansat

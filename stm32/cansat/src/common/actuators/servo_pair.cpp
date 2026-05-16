#include "cansat/actuators/servo_pair.hpp"

#include <Arduino.h>
#include <HardwareTimer.h>

namespace cansat {

namespace {

// PE11 = TIM1_CH2 (right servo)
// PE13 = TIM1_CH3 (left  servo)
constexpr uint8_t kRightChannel = 2;
constexpr uint8_t kLeftChannel  = 3;

}  // namespace

void ServoPair::begin() {
  if (timer_ != nullptr) {
    return;
  }
  // Function-local static: constructed once on first call, lives for the rest
  // of the program. Avoids heap allocation and the static-init-order issue
  // that file-scope statics would have on stm32duino (HAL not yet up).
  static HardwareTimer instance(TIM1);
  timer_ = &instance;

  timer_->setMode(kRightChannel, TIMER_OUTPUT_COMPARE_PWM1, PE11);
  timer_->setMode(kLeftChannel,  TIMER_OUTPUT_COMPARE_PWM1, PE13);

  timer_->setOverflow(kPeriodUs, MICROSEC_FORMAT);
  timer_->setCaptureCompare(kRightChannel, kRestUs, MICROSEC_COMPARE_FORMAT);
  timer_->setCaptureCompare(kLeftChannel,  kRestUs, MICROSEC_COMPARE_FORMAT);

  timer_->resume();  // enables MOE on advanced timer (TIM1) automatically
  ready_ = true;
}

uint16_t ServoPair::pull_to_us(float pull) {
  if (pull < 0.0f) {
    pull = 0.0f;
  }
  if (pull > 1.0f) {
    pull = 1.0f;
  }
  // kFullPullUs is already bounded by kMaxAngleDeg in the header, so pull=1.0
  // can never exceed the configured angle limit.
  const float span = static_cast<float>(kFullPullUs) - static_cast<float>(kRestUs);
  return static_cast<uint16_t>(static_cast<float>(kRestUs) + pull * span);
}

uint16_t ServoPair::angle_deg_to_us(float angle_deg) {
  const float max_deg = static_cast<float>(kMaxAngleDeg);
  if (angle_deg < 0.0f) {
    angle_deg = 0.0f;
  }
  if (angle_deg > max_deg) {
    angle_deg = max_deg;  // hard ceiling: never command beyond physical limit
  }
  const float us = static_cast<float>(kRestUs)
                 + angle_deg * (static_cast<float>(kUsPerDegX100) / 100.0f);
  return static_cast<uint16_t>(us);
}

void ServoPair::set_left(float pull) {
  if (!ready_) {
    return;
  }
  timer_->setCaptureCompare(kLeftChannel, pull_to_us(pull), MICROSEC_COMPARE_FORMAT);
}

void ServoPair::set_right(float pull) {
  if (!ready_) {
    return;
  }
  timer_->setCaptureCompare(kRightChannel, pull_to_us(pull), MICROSEC_COMPARE_FORMAT);
}

void ServoPair::set_pair(float left_pull, float right_pull) {
  set_left(left_pull);
  set_right(right_pull);
}

void ServoPair::set_left_angle_deg(float angle_deg) {
  if (!ready_) {
    return;
  }
  timer_->setCaptureCompare(kLeftChannel, angle_deg_to_us(angle_deg), MICROSEC_COMPARE_FORMAT);
}

void ServoPair::set_right_angle_deg(float angle_deg) {
  if (!ready_) {
    return;
  }
  timer_->setCaptureCompare(kRightChannel, angle_deg_to_us(angle_deg), MICROSEC_COMPARE_FORMAT);
}

void ServoPair::neutral() { set_pair(0.0f, 0.0f); }

}  // namespace cansat

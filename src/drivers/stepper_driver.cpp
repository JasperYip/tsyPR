#include "drivers/stepper_driver.hpp"
#include <Arduino.h>

StepperDriver::StepperDriver(const Config& cfg) : cfg_(cfg) {}

void StepperDriver::begin() {
  pinMode(cfg_.pin_step, OUTPUT);
  pinMode(cfg_.pin_dir, OUTPUT);
  pinMode(cfg_.pin_en, OUTPUT);
  pinMode(cfg_.pin_flt, INPUT_PULLUP);

  digitalWrite(cfg_.pin_step, LOW);
  digitalWrite(cfg_.pin_dir, LOW);

  // Default disabled
  if (cfg_.en_active_low) {
    digitalWrite(cfg_.pin_en, HIGH);
  } else {
    digitalWrite(cfg_.pin_en, LOW);
  }

  enabled_ = false;
  dir_ = Direction::CW;
  steps_per_sec_ = cfg_.steps_per_sec;
}

void StepperDriver::enable() {
  digitalWrite(cfg_.pin_en, cfg_.en_active_low ? LOW : HIGH);
  enabled_ = true;
}

void StepperDriver::disable() {
  digitalWrite(cfg_.pin_en, cfg_.en_active_low ? HIGH : LOW);
  enabled_ = false;
}

void StepperDriver::setDirection(Direction dir) {
  dir_ = dir;
  digitalWrite(cfg_.pin_dir, (dir_ == Direction::CCW) ? HIGH : LOW);
}

void StepperDriver::pulseStep_(uint32_t interval_us) const {
  const uint32_t pulse = static_cast<uint32_t>(cfg_.step_pulse_us);

  digitalWrite(cfg_.pin_step, HIGH);
  delayMicroseconds(pulse);

  digitalWrite(cfg_.pin_step, LOW);
  delayMicroseconds(interval_us - pulse);
}

void StepperDriver::step() {
  if (!enabled_) return;

  const uint32_t interval =
      static_cast<uint32_t>(1e6f / steps_per_sec_);

  pulseStep_(interval);
}

void StepperDriver::step(uint32_t steps) {
  if (!enabled_) return;

  const uint32_t interval =
      static_cast<uint32_t>(1e6f / steps_per_sec_);

  for (uint32_t i = 0; i < steps; ++i) {
    pulseStep_(interval);
  }
}

void StepperDriver::move(uint32_t steps) {
  if (!enabled_) return;

  const float min_speed = 50.0f;
  const float max_speed = steps_per_sec_;

  for (uint32_t i = 0; i < steps; ++i) {

    float progress = (float)i / steps;

    float speed;
    if (progress < 0.2f) {
      // accelerate
      speed = min_speed +
              (max_speed - min_speed) * (progress / 0.2f);
    }
    else if (progress > 0.8f) {
      // decelerate
      speed = min_speed +
              (max_speed - min_speed) *
              ((1.0f - progress) / 0.2f);
    }
    else {
      // cruise
      speed = max_speed;
    }

    const uint32_t interval =
        static_cast<uint32_t>(1e6f / speed);

    pulseStep_(interval);
  }
}

void StepperDriver::setSpeed(float steps_per_sec) {
  if (steps_per_sec < 1.0f) steps_per_sec = 1.0f;
  steps_per_sec_ = steps_per_sec;
}

bool StepperDriver::faultActive() const {
  const bool raw_high = (digitalRead(cfg_.pin_flt) == HIGH);
  return cfg_.flt_active_low ? !raw_high : raw_high;
}
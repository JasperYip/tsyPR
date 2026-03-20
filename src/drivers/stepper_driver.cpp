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

  // Default disabled (safe)
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
  if (cfg_.en_active_low) {
    digitalWrite(cfg_.pin_en, LOW);
  } else {
    digitalWrite(cfg_.pin_en, HIGH);
  }
  enabled_ = true;
}

void StepperDriver::disable() {
  if (cfg_.en_active_low) {
    digitalWrite(cfg_.pin_en, HIGH);
  } else {
    digitalWrite(cfg_.pin_en, LOW);
  }
  enabled_ = false;
}

void StepperDriver::setDirection(Direction dir) {
  dir_ = dir;
  digitalWrite(cfg_.pin_dir, (dir_ == Direction::CW) ? HIGH : LOW);
}

void StepperDriver::pulseStep_() const {
  digitalWrite(cfg_.pin_step, HIGH);
  delayMicroseconds(static_cast<uint32_t>(cfg_.step_pulse_us));
  digitalWrite(cfg_.pin_step, LOW);
}

void StepperDriver::step() {
  if (!enabled_) return;
  pulseStep_();
}

void StepperDriver::step(uint32_t steps) {
  if (!enabled_) return;

  const float delay_us = 1e6f / steps_per_sec_;

  for (uint32_t i = 0; i < steps; ++i) {
    pulseStep_();
    delayMicroseconds(static_cast<uint32_t>(delay_us));
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
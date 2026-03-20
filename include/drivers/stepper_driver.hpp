#pragma once
#include <stdint.h>

class StepperDriver {
public:
  enum class Direction : uint8_t {
    CW = 0,
    CCW = 1
  };

  struct Config {
    uint8_t pin_step;
    uint8_t pin_dir;
    uint8_t pin_en;
    uint8_t pin_flt;

    // Timing
    float step_pulse_us = 2.0f;     // minimum pulse width (datasheet ~1.9us)
    float steps_per_sec = 1000.0f;  // default speed

    // Fault polarity (often active-low)
    bool flt_active_low = true;

    // Enable polarity (depends on board: DRV8825 = LOW enabled)
    bool en_active_low = true;
  };

  explicit StepperDriver(const Config& cfg);

  void begin();

  void enable();
  void disable();

  void setDirection(Direction dir);
  Direction direction() const { return dir_; }

  // Single step
  void step();

  // Multiple steps (blocking)
  void step(uint32_t steps);

  // Speed control
  void setSpeed(float steps_per_sec);
  float speed() const { return steps_per_sec_; }

  bool faultActive() const;
  bool enabled() const { return enabled_; }

private:
  Config cfg_;
  Direction dir_ = Direction::CW;
  bool enabled_ = false;

  float steps_per_sec_ = 1000.0f;

  void pulseStep_() const;
};
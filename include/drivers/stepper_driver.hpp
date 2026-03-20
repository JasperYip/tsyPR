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

    float step_pulse_us = 5.0f;     // safer default
    float steps_per_sec = 100.0f;   // safe startup speed

    bool flt_active_low = true;
    bool en_active_low  = true;
  };

  explicit StepperDriver(const Config& cfg);

  void begin();

  void enable();
  void disable();

  void setDirection(Direction dir);
  Direction direction() const { return dir_; }

  // Constant speed
  void step();
  void step(uint32_t steps);

  // Motion with acceleration
  void move(uint32_t steps);

  // Speed control
  void setSpeed(float steps_per_sec);
  float speed() const { return steps_per_sec_; }

  bool faultActive() const;
  bool enabled() const { return enabled_; }

private:
  Config cfg_;
  Direction dir_ = Direction::CW;
  bool enabled_ = false;

  float steps_per_sec_ = 100.0f;

  void pulseStep_(uint32_t interval_us) const;
};
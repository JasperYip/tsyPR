#pragma once
#include <stdint.h>

class HallSensor {
public:
  struct Config {
    uint8_t pin;
    bool active_low = true;
    bool use_pullup = true;
  };

  explicit HallSensor(const Config& cfg);

  void begin();
  bool rawHigh() const;
  bool isActive() const; // true when hall target is detected

private:
  Config cfg_;
};


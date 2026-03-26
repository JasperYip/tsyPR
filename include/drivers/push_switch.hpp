#pragma once
#include <stdint.h>

class PushSwitch {
public:
  struct Config {
    uint8_t pin;
    bool active_low = true;
    bool use_pullup = true;
  };

  explicit PushSwitch(const Config& cfg);

  void begin();
  bool rawHigh() const;
  bool isPressed() const;

private:
  Config cfg_;
};


#pragma once
#include <stdint.h>

// We implemented a simple LeakSensor driver that wraps a digital 
// input pin and exposes a clean isLeak() interface, abstracting 
// whether the signal is active-high or active-low. The driver 
// initializes the pin (optionally with pull-up) and provides 
// both a raw read and a logical leak-detected result, keeping 
// hardware details isolated from higher-level safety logic. 
// A quick smoke test confirmed stable LOW in dry conditions and 
// HIGH when a leak is triggered, validating correct wiring 
// and polarity.

class LeakSensor {
public:
  // active_high = true means pin reads HIGH when leak is detected (your case)
  explicit LeakSensor(uint8_t pin, bool active_high = true);

  void begin(bool use_pullup = false);
  bool isLeak() const;     // true when leak detected
  bool raw() const;        // raw digital read

private:
  uint8_t pin_;
  bool active_high_;
};
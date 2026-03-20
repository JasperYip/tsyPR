#include "drivers/leak_sensor.hpp"
#include <Arduino.h>

LeakSensor::LeakSensor(uint8_t pin, bool active_high)
: pin_(pin), active_high_(active_high) {}

void LeakSensor::begin(bool use_pullup) {
  pinMode(pin_, use_pullup ? INPUT_PULLUP : INPUT);
}

bool LeakSensor::raw() const {
  return digitalRead(pin_) == HIGH;
}

bool LeakSensor::isLeak() const {
  const bool r = raw();
  return active_high_ ? r : !r;
}
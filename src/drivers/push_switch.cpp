#include "drivers/push_switch.hpp"
#include <Arduino.h>

PushSwitch::PushSwitch(const Config& cfg) : cfg_(cfg) {}

void PushSwitch::begin() {
  pinMode(cfg_.pin, cfg_.use_pullup ? INPUT_PULLUP : INPUT);
}

bool PushSwitch::rawHigh() const {
  return digitalRead(cfg_.pin) == HIGH;
}

bool PushSwitch::isPressed() const {
  const bool high = rawHigh();
  return cfg_.active_low ? !high : high;
}


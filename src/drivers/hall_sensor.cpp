#include "drivers/hall_sensor.hpp"
#include <Arduino.h>

HallSensor::HallSensor(const Config& cfg) : cfg_(cfg) {}

void HallSensor::begin() {
  pinMode(cfg_.pin, cfg_.use_pullup ? INPUT_PULLUP : INPUT);
}

bool HallSensor::rawHigh() const {
  return digitalRead(cfg_.pin) == HIGH;
}

bool HallSensor::isActive() const {
  const bool high = rawHigh();
  return cfg_.active_low ? !high : high;
}


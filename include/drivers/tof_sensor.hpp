#pragma once
#include <stdint.h>
#include <Wire.h>

/*
We built a dedicated ToFSensor driver that runs on I2C1 (Wire1), 
initializes the VL6180X, and provides a clean read() interface 
returning a structured result (distance in mm, validity flag, 
and status code). The driver performs a small number of internal 
retries when a reading is invalid (status ≠ 0), and if all retries 
fail it returns the last valid distance instead of garbage, along 
with flags indicating the data is not fresh. This keeps the hardware 
layer clean, isolates sensor quirks, and ensures higher-level 
controllers receive stable, well-defined measurements.
*/

class ToFSensor {
public:
  struct Reading {
    bool      fresh;
    bool      valid;
    uint16_t  mm;
    uint8_t   status;
  };

  // ✅ Constructor stores pins
  ToFSensor(int sda_pin, int scl_pin)
    : sda_pin_(sda_pin), scl_pin_(scl_pin) {}

  bool begin(TwoWire& wire = Wire1);

  void setMinValidMm(uint16_t mm) { min_valid_mm_ = mm; }

  Reading read();

private:
  int sda_pin_;
  int scl_pin_;

  bool inited_ = false;
  uint16_t last_valid_mm_ = 0;
  uint16_t min_valid_mm_  = 8;

  float offset_mm_ = 0.0f; // private correction
};
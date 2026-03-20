#include "drivers/tof_sensor.hpp"
#include <Adafruit_VL6180X.h>

static Adafruit_VL6180X g_vl;

bool ToFSensor::begin(TwoWire& wire) {
  wire.setSDA(sda_pin_);
  wire.setSCL(scl_pin_);
  
  wire.begin();
  wire.setClock(400000);

  if (!g_vl.begin(&wire)) {
    inited_ = false;
    return false;
  }
  inited_ = true;
  return true;
}

ToFSensor::Reading ToFSensor::read() {
  Reading out{};
  if (!inited_) {
    out.fresh = false;
    out.valid = false;
    out.status = 0xFF;
    out.mm = last_valid_mm_;
    return out;
  }

  const int MAX_RETRIES = 3;
  uint8_t status = 0xFF;
  uint8_t range  = 0;

  for (int i = 0; i < MAX_RETRIES; i++) {
    range  = g_vl.readRange();
    status = g_vl.readRangeStatus();

    const uint16_t mm = static_cast<uint16_t>(range);

    // Accept only if status OK AND above your minimum reliable floor
    if (status == 0 && mm >= min_valid_mm_) {
      const float corrected = static_cast<float>(mm) + offset_mm_;

      // Prevent negative values after correction
      const uint16_t corrected_mm = (corrected < 0.0f) ? 0 : static_cast<uint16_t>(corrected);

      out.valid = true;
      out.fresh = true;
      out.mm = corrected_mm;
      out.status = status;

      last_valid_mm_ = corrected_mm;
      return out;
    }
  }

  // All retries failed or below min range → return last valid
  out.valid  = false;
  out.fresh  = false;
  out.status = status;
  out.mm     = last_valid_mm_;
  return out;
}
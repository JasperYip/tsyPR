#include <Arduino.h>
#include "drivers/stepper_driver.hpp"
#include "config/pin_map.hpp"

// ===== Roll Motor =====
StepperDriver roll({
  .pin_step = PIN_ROLL_STEP,
  .pin_dir  = PIN_ROLL_DIR,
  .pin_en   = PIN_ROLL_ENA,
  .pin_flt  = PIN_ROLL_FLT,

  .step_pulse_us = 3.0f,
  .steps_per_sec = 800.0f,

  .flt_active_low = true,
  .en_active_low  = true   // ⚠️ confirm this!
});

// ===== Pitch Motor =====
StepperDriver pitch({
  .pin_step = PIN_PITCH_STEP,
  .pin_dir  = PIN_PITCH_DIR,
  .pin_en   = PIN_PITCH_ENA,
  .pin_flt  = PIN_PITCH_FLT,

  .step_pulse_us = 3.0f,
  .steps_per_sec = 800.0f,

  .flt_active_low = true,
  .en_active_low  = true
});

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Dual Stepper Test Start");

  roll.begin();
  pitch.begin();

  roll.enable();
  pitch.enable();

  if (roll.faultActive())  Serial.println("⚠️ Roll fault!");
  if (pitch.faultActive()) Serial.println("⚠️ Pitch fault!");
}

void loop() {

  Serial.println("PITCH CW 1000");
  pitch.setDirection(StepperDriver::Direction::CW);
  pitch.step(1000);

  delay(2000);

  Serial.println("PITCH CCW 1000");
  pitch.setDirection(StepperDriver::Direction::CCW);
  pitch.step(1000);

  delay(3000);
}
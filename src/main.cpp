#include <Arduino.h>
#include "drivers/stepper_driver.hpp"
#include "config/pin_map.hpp"
#include "config/constants.hpp"

uint32_t lastPrint = 0;
const uint32_t PRINT_INTERVAL_MS = 1000;

// forward declaration
void printStatus();

// ===== Stepper =====
StepperDriver pitch({
  .pin_step = PIN_PITCH_STEP,
  .pin_dir  = PIN_PITCH_DIR,
  .pin_en   = PIN_PITCH_ENA,
  .pin_flt  = PIN_PITCH_FLT,

  .step_pulse_us = 5.0f,
  .steps_per_sec = 1500.0f,

  .flt_active_low = true,
  .en_active_low  = true
});

// ===== State =====
StepperDriver::Direction currentDir = StepperDriver::Direction::CW;
int32_t current_steps = 0;
String cmdBuffer;

// ===== Helpers =====
int32_t mmToSteps(float mm) {
  return (int32_t)(mm / config::PITCH_STEP_MM);
}

float stepsToMm(int32_t steps) {
  return steps * config::PITCH_STEP_MM;
}

// ===== Command Handling =====
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "f") {
    currentDir = StepperDriver::Direction::CW;
    pitch.setDirection(currentDir);
    Serial.println("Direction: Forward");
  }
  else if (cmd == "b") {
    currentDir = StepperDriver::Direction::CCW;
    pitch.setDirection(currentDir);
    Serial.println("Direction: Backward");
  }
  else if (cmd.startsWith("m")) {
    float mm = cmd.substring(1).toFloat();

    if (mm <= 0.0f) {
      Serial.println("Invalid move");
      return;
    }

    int32_t steps = mmToSteps(mm);

    if (currentDir == StepperDriver::Direction::CW) {
      current_steps += steps;
    } else {
      current_steps -= steps;
    }

    Serial.print("Move ");
    Serial.print(mm);
    Serial.print(" mm (");
    Serial.print(steps);
    Serial.println(" steps)");

    pitch.move(steps);

    Serial.print("New position: ");
    Serial.print(stepsToMm(current_steps), 3);
    Serial.println(" mm");
  }
  else if (cmd == "pos") {
    Serial.print("Position: ");
    Serial.print(stepsToMm(current_steps), 3);
    Serial.println(" mm");
  }
  else if (cmd == "zero") {
    current_steps = 0;
    Serial.println("Position reset to 0");
  }
  else if (cmd == "status") {
    printStatus();
  }
  else {
    Serial.println("Unknown command");
  }
}

// ===== Serial Reader =====
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }
}

// ===== Status =====
void printStatus() {
  Serial.print("pos_mm=");
  Serial.print(stepsToMm(current_steps), 3);

  Serial.print("  steps=");
  Serial.print(current_steps);

  Serial.print("  dir=");
  Serial.print(currentDir == StepperDriver::Direction::CW ? "FWD" : "BWD");

  Serial.print("  fault=");
  Serial.print(pitch.faultActive() ? "YES" : "NO");

  Serial.println();
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Stepper Position Control");

  pitch.begin();
  pitch.enable();
  pitch.setDirection(currentDir);
  pitch.setSpeed(1500);

  if (pitch.faultActive()) {
    Serial.println("Driver fault!");
  } else {
    Serial.println("Driver OK");
  }

  Serial.println("\nCommands:");
  Serial.println("f       -> forward");
  Serial.println("b       -> backward");
  Serial.println("m<num>  -> move <num> mm");
  Serial.println("pos     -> print position");
  Serial.println("status  -> print full status");
  Serial.println("zero    -> reset position\n");
}

// ===== Loop =====
void loop() {
  readSerial();

  uint32_t now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    printStatus();
  }
}
#pragma once

/*
------------------------------------------------------------
P&R TEENSY SYSTEM CONSTANTS
------------------------------------------------------------

All global system configuration values live here.

Rules:
- No hardware pins here (pin_map.hpp handles that)
- No driver-specific constants
- Only system behaviour parameters

Units are explicitly stated to avoid ambiguity.
------------------------------------------------------------
*/

#include <stdint.h>

namespace config
{

/* ----------------------------------------------------------
   NODE CONFIGURATION
---------------------------------------------------------- */

constexpr uint8_t NODE_ID_LEFT  = 3;

/* ----------------------------------------------------------
   STEP GEOMETRY
---------------------------------------------------------- */

constexpr float PITCH_STEP_MM = 0.01f;
constexpr float ROLL_STEP_DEG = 0.045f;

constexpr float PITCH_MAX_MM = 56.9f;
constexpr float ROLL_MAX_DEG = 90.0f;
constexpr float PITCH_MIN_MM = -PITCH_MAX_MM;
constexpr float ROLL_MIN_DEG = -ROLL_MAX_DEG;

/* ----------------------------------------------------------
   SAFETY PLACEHOLDERS
---------------------------------------------------------- */

constexpr float OVERCURRENT_HARD_LIMIT_A = 6.0f;
constexpr float OVERCURRENT_CONFIRM_S = 0.50f;
constexpr float STALL_CURRENT_A = 3.0f;
constexpr float STALL_MIN_DUTY = 0.25f;
constexpr float STALL_POS_EPS_MM = 0.10f;
constexpr float STALL_CONFIRM_S = 2.0f;
constexpr float BMS_TEMP_CONFIRM_S = 2.0f;
constexpr float HOMING_TIMEOUT_S = 60.0f;

// debounce
constexpr float LEAK_DEBOUNCE_S = 0.05f;
constexpr float DRIVER_FAULT_DEBOUNCE_S = 0.02f;

// command timeout
constexpr uint32_t CMD_TIMEOUT_SMALL_MS = 300;
constexpr uint32_t CMD_TIMEOUT_LARGE_MS = 120000;


// BMS timeout
constexpr uint32_t BMS_TIMEOUT_SMALL_MS = 5000;
constexpr uint32_t BMS_TIMEOUT_LARGE_MS = 120000;

}

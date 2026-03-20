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



}
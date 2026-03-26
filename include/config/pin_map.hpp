#pragma once
#include <Arduino.h>

// =====================
// VBD Teensy Pin Map
// =====================
//
// Notes:
// Notes:
// - CAN2 uses pins 0 (RX) and 1 (TX) on Teensy 4.x
// - I2C1 uses pins 16 (SDA1) and 17 (SCL1) => Wire1
// - BMS UART on pins 7/8 (typical Teensy Serial2 mapping)
// - Proximity is pull-up (likely active-low when triggered)

// =====================
// Node identity
// =====================
#ifndef NODE_ID
#define NODE_ID 1 // fallback if not defined
#endif

// =====================
// CAN (Transceiver)
// =====================
// You said: 0 -> R, 1 -> D on CAN transceiver.
// Keep names generic: Teensy TX/RX to transceiver DI/RO depends on your transceiver.
// We'll just label what you told me.
// CAN2 uses TX=1, RX=0
constexpr uint8_t PIN_CAN_R = 0;   // connected to transceiver R (RO)
constexpr uint8_t PIN_CAN_D = 1;   // connected to transceiver D (DI)

// =====================
// Roll Driver (DRV8825)
// =====================
constexpr uint8_t PIN_PITCH_DIR = 2;   // Pitch motor direction
constexpr uint8_t PIN_PITCH_STEP = 3;   // Pitch motor step
constexpr uint8_t PIN_PITCH_ENA = 4;   // Pitch motor enable ()
constexpr uint8_t PIN_PITCH_FLT  = 5;  // Pitch motor fault

// =====================
// BMS UART (TinyBMS)
// =====================
constexpr uint8_t PIN_BMS_TX = 7;  // Teensy TX -> BMS RX (check direction in wiring)
constexpr uint8_t PIN_BMS_RX = 8;  // Teensy RX <- BMS TX
constexpr uint32_t BMS_BAUD  = 115200;

// =====================
// Limit Switch 
// =====================
constexpr uint8_t PIN_FWD_SWH = 9;  // Fwd limit switch
constexpr uint8_t PIN_BWD_SWH = 10;  // Bwd limit switch

// =====================
// Hall Effect Sensor
// =====================
constexpr uint8_t PIN_HALL_SENS = 15;  // Hall effect GPIO

// =====================
// VL6180X ToF (I2C)
// =====================
constexpr uint8_t PIN_TOF_SCL = 16;
constexpr uint8_t PIN_TOF_SDA = 17;

// =====================
// Leak sensor
// =====================
constexpr uint8_t PIN_LEAK = 18;

// =====================
// Pitch Driver (DRV8825)
// =====================
constexpr uint8_t PIN_ROLL_DIR = 20;   // Roll motor direction
constexpr uint8_t PIN_ROLL_STEP = 21;   // Roll motor step
constexpr uint8_t PIN_ROLL_ENA = 22;   // Roll motor enable ()
constexpr uint8_t PIN_ROLL_FLT  = 23;  // Roll motor fault

#pragma once

/*
------------------------------------------------------------
CAN MESSAGE DEFINITIONS
------------------------------------------------------------

This file defines:

- CAN IDs
- Message structs
- Bitfield definitions
- Pack / unpack interfaces

All CAN messages are fixed 8-byte frames.

------------------------------------------------------------
*/

#include <stdint.h>

namespace can
{

/* ----------------------------------------------------------
   CAN BASE IDS
---------------------------------------------------------- */

constexpr uint16_t STATUS_FAULT_BASE   = 0x053;
constexpr uint16_t CMD_SETPOINT_BASE   = 0x103;
constexpr uint16_t STATUS_CONTROL_BASE = 0x203;
constexpr uint16_t STATUS_BMS_BASE     = 0x213;


/* ----------------------------------------------------------
   SYSTEM STATE
---------------------------------------------------------- */

enum SystemState : uint8_t
{
    STATE_UNHOMED     = 0,
    STATE_HOMING      = 1,
    STATE_RUN         = 2,
    STATE_HOLD        = 3,
    STATE_FAULT       = 4
};


/* ----------------------------------------------------------
   HARD FAULT BITFIELDS
---------------------------------------------------------- */

enum HardFaultA : uint8_t
{
    FAULT_LEAK             = 1 << 0,
    FAULT_DRIVER_P         = 1 << 1,
    FAULT_DRIVER_R         = 1 << 2,
    FAULT_CMD_TO_L         = 1 << 3,
    FAULT_HOMING_FAILED_P  = 1 << 4,
    FAULT_HOMING_FAILED_R  = 1 << 5,
    FAULT_BMS_TEMP         = 1 << 6,
    FAULT_BMS_TO_L         = 1 << 7
};

enum HardFaultB : uint8_t
{
    FAULT_BMS_OC        = 1 << 0,
    FAULT_BMS_SWITCH    = 1 << 1,
    FAULT_STALL_P       = 1 << 2,
    FAULT_STALL_R       = 1 << 3
};


/* ----------------------------------------------------------
   SOFT FAULT BITFIELDS
---------------------------------------------------------- */

enum SoftFault : uint8_t
{
    FAULT_CMD_TO      = 1 << 0,
    FAULT_TOF_INV     = 1 << 1,
    FAULT_BMS_TO      = 1 << 2,
    FAULT_BMS_LOW     = 1 << 3,
    FAULT_BMS_HIGH    = 1 << 4
};


/* ----------------------------------------------------------
   COMMAND MODE BITFIELD
---------------------------------------------------------- */

enum CommandMode : uint8_t
{
    CMD_MOTOR_ENABLE = 1 << 0,
    CMD_START_HOMING = 1 << 1
};


/* ----------------------------------------------------------
   STATUS_CONTROL FLAG BITFIELD
---------------------------------------------------------- */

enum ControlFlagsA : uint8_t
{
    FLAG_DIR_P        = 1 << 0,
    FLAG_DIR_R        = 1 << 1,
    FLAG_HOMED_P      = 1 << 2,
    FLAG_HOMED_R      = 1 << 3,
    FLAG_MOVING_P     = 1 << 4,
    FLAG_MOVING_R     = 1 << 5,
    FLAG_ENABLE_P     = 1 << 6,
    FLAG_ENABLE_R     = 1 << 7
};

enum ControlFlagsB : uint8_t
{
    FLAG_LIM_F        = 1 << 0,
    FLAG_LIM_R        = 1 << 1,
    FLAG_LEAK         = 1 << 2,
    FLAG_HALL         = 1 << 3
};

/* ----------------------------------------------------------
   MESSAGE STRUCTS
---------------------------------------------------------- */

struct StatusFault
{
    uint8_t hard_fault_a;
    uint8_t hard_fault_b;
    uint8_t soft_fault;
    uint8_t reserved0;

    uint8_t first_hard_fault;
    uint8_t state;

    uint8_t sequence;
    uint8_t reserved1;
};


struct CmdSetpoint
{
    int16_t pitch_travel_01mm;
    int16_t roll_angle_01deg;

    uint8_t command_mode;

    uint8_t reserved0;

    uint8_t sequence;

    uint8_t reserved1;
};


struct StatusControl
{
    int16_t pitch_pos_01mm;
    int16_t roll_angle_01deg;

    uint8_t flagsA;
    uint8_t flagsB;

    uint8_t sequence;

    int8_t tof_mm;
};


struct StatusBMS
{
    uint16_t pack_mV;       // battery pack voltage (mV)
    int16_t pack_temp;      // battery pack temperature (degC)

    uint8_t bms_fault_id;   // raw fault ID reported by TinyBMS (see BMS datasheet table)

    uint8_t reserved0;

    uint8_t sequence;       // rolling counter

    uint8_t reserved1;
};


/* ----------------------------------------------------------
   PACKING FUNCTIONS
---------------------------------------------------------- */

void packStatusFault(uint8_t *buf, const StatusFault &msg);
void packCmdSetpoint(uint8_t *buf, const CmdSetpoint &msg);
void packStatusControl(uint8_t *buf, const StatusControl &msg);
void packStatusBMS(uint8_t *buf, const StatusBMS &msg);

void unpackCmdSetpoint(const uint8_t *buf, CmdSetpoint &msg);

}
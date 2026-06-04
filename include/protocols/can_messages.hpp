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

constexpr uint16_t STATUS_FAULT_BASE    = 0x050;
constexpr uint16_t CMD_SETPOINT_BASE    = 0x100;
constexpr uint16_t STATUS_CONTROL_BASE  = 0x200;
constexpr uint16_t STATUS_BMS_BASE      = 0x210;
constexpr uint16_t STATUS_BMS_MORE_BASE = 0x220;


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
    CMD_MOTOR_ENABLE  = 1 << 0,
    CMD_START_HOMING  = 1 << 1,
    CMD_FAULT_CLEAR   = 1 << 2   // clear latched faults and return to CAN mode
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


/*
------------------------------------------------------------
STATUS_BMS  (0x213)
------------------------------------------------------------
Encoding:
  pack_V:       uint8  ×0.1 V       → decode: value / 10.0
  min/max_cell: uint8  (mV−2500)/10 → decode: value * 10 + 2500
  pack_current: int8   ×0.5 A       → decode: value * 0.5  (+ = discharge)
  temp_*:       uint8  °C           → 0xFF = invalid / not connected
------------------------------------------------------------
*/
struct StatusBMS
{
    uint8_t pack_V;         // pack voltage ×0.1V  (e.g. 168 = 16.8 V)
    uint8_t min_cell_V;     // (mV − 2500) / 10    (e.g.  81 = 3310 mV)
    uint8_t max_cell_V;     // (mV − 2500) / 10    (e.g. 172 = 4220 mV)
    int8_t  pack_current;   // ×0.5 A, + = discharge (e.g. −17 = −8.5 A charging)
    uint8_t temp_internal;  // °C, 0xFF = invalid
    uint8_t temp_ext1;      // °C, 0xFF = not connected
    uint8_t temp_ext2;      // °C, 0xFF = not connected
    uint8_t sequence;
};


/*
------------------------------------------------------------
STATUS_BMS_MORE  (0x223)
------------------------------------------------------------
cell_V[0..5]: first 6 individual cell voltages from BMS cmd 0x1C.
              Encoding: (mV − 2500) / 10, same as min/max_cell_V.
              Unused slots (fewer than 6 cells) remain 0.
online_status: 0=Unknown 1=Charging 2=FullyCharged
               3=Discharging 4=Idle 5=Fault
------------------------------------------------------------
*/
struct StatusBmsMore
{
    uint8_t cell_V[6];      // individual cells, same encoding as min/max_cell_V
    uint8_t online_status;  // compact 0-5 enum (see above)
    uint8_t sequence;
};


/* ----------------------------------------------------------
   PACKING FUNCTIONS
---------------------------------------------------------- */

void packStatusFault(uint8_t *buf, const StatusFault &msg);
void packCmdSetpoint(uint8_t *buf, const CmdSetpoint &msg);
void packStatusControl(uint8_t *buf, const StatusControl &msg);
void packStatusBMS(uint8_t *buf, const StatusBMS &msg);
void packStatusBmsMore(uint8_t *buf, const StatusBmsMore &msg);

void unpackCmdSetpoint(const uint8_t *buf, CmdSetpoint &msg);

}
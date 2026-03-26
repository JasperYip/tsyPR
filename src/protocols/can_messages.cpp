#include "protocols/can_messages.hpp"

namespace can
{

/*
------------------------------------------------------------
STATUS_FAULT
------------------------------------------------------------
*/
void packStatusFault(uint8_t *buf, const StatusFault &m)
{
    buf[0] = m.hard_fault_a;
    buf[1] = m.hard_fault_b;
    buf[2] = m.soft_fault;
    buf[3] = 0;

    buf[4] = m.first_hard_fault;
    buf[5] = m.state;

    buf[6] = m.sequence;
    buf[7] = 0;
}


/*
------------------------------------------------------------
CMD_SETPOINT
------------------------------------------------------------
*/
void packCmdSetpoint(uint8_t *buf, const CmdSetpoint &m)
{
    buf[0] = m.pitch_travel_01mm & 0xFF;
    buf[1] = m.pitch_travel_01mm >> 8;

    buf[2] = m.roll_angle_01deg & 0xFF;
    buf[3] = m.roll_angle_01deg >> 8;

    buf[4] = m.command_mode;
    buf[5] = 0;

    buf[6] = m.sequence;

    buf[7] = 0;
}

void unpackCmdSetpoint(const uint8_t *buf, CmdSetpoint &m)
{
    m.pitch_travel_01mm =
        static_cast<int16_t>(
            static_cast<uint16_t>(buf[0]) |
            (static_cast<uint16_t>(buf[1]) << 8));

    m.roll_angle_01deg =
        static_cast<int16_t>(
            static_cast<uint16_t>(buf[2]) |
            (static_cast<uint16_t>(buf[3]) << 8));

    m.command_mode = buf[4];
    m.reserved0 = 0;

    m.sequence = buf[6];
    m.reserved1 = 0;
}


/*
------------------------------------------------------------
STATUS_CONTROL
------------------------------------------------------------
*/
void packStatusControl(uint8_t *buf, const StatusControl &m)
{
    buf[0] = m.pitch_pos_01mm & 0xFF;
    buf[1] = m.pitch_pos_01mm >> 8;

    buf[2] = m.roll_angle_01deg & 0xFF;
    buf[3] = m.roll_angle_01deg >> 8;

    buf[4] = m.flagsA;
    buf[5] = m.flagsB;

    buf[6] = m.sequence;

    buf[7] = m.tof_mm;
}


/*
------------------------------------------------------------
STATUS_BMS
------------------------------------------------------------
*/
void packStatusBMS(uint8_t *buf, const StatusBMS &m)
{
    buf[0] = m.pack_mV & 0xFF;
    buf[1] = m.pack_mV >> 8;

    buf[2] = m.pack_temp & 0xFF;
    buf[3] = m.pack_temp >> 8;

    buf[4] = m.bms_fault_id;   // raw TinyBMS fault code

    buf[5] = 0;

    buf[6] = m.sequence;

    buf[7] = 0;
}

}

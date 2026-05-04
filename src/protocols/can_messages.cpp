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
STATUS_BMS  (0x213)
------------------------------------------------------------
Encoding matches tsyVBDL STATUS_BMS (0x210):
  [0] pack_V       uint8 ×0.1V
  [1] min_cell_V   uint8 (mV−2500)/10
  [2] max_cell_V   uint8 (mV−2500)/10
  [3] pack_current int8  ×0.5A  (+ = discharge)
  [4] temp_internal uint8 °C, 0xFF=invalid
  [5] temp_ext1    uint8 °C, 0xFF=not connected
  [6] temp_ext2    uint8 °C, 0xFF=not connected
  [7] sequence
------------------------------------------------------------
*/
void packStatusBMS(uint8_t *buf, const StatusBMS &m)
{
    buf[0] = m.pack_V;
    buf[1] = m.min_cell_V;
    buf[2] = m.max_cell_V;
    buf[3] = static_cast<uint8_t>(m.pack_current);
    buf[4] = m.temp_internal;
    buf[5] = m.temp_ext1;
    buf[6] = m.temp_ext2;
    buf[7] = m.sequence;
}


/*
------------------------------------------------------------
STATUS_BMS_MORE  (0x223)
------------------------------------------------------------
Encoding matches tsyVBDL STATUS_BMS_MORE (0x220):
  [0..5] cell_V[6]   uint8 (mV−2500)/10
         [0]=min [1]=max [2..5]=0 (individual cells not polled)
  [6]    online_status  0=Unknown 1=Charging 2=FullyCharged
                        3=Discharging 4=Idle 5=Fault
  [7]    sequence
------------------------------------------------------------
*/
void packStatusBmsMore(uint8_t *buf, const StatusBmsMore &m)
{
    for (int i = 0; i < 6; i++) buf[i] = m.cell_V[i];
    buf[6] = m.online_status;
    buf[7] = m.sequence;
}

}

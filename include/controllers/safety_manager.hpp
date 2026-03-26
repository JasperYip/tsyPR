#pragma once

#include <stdint.h>

#include "config/constants.hpp"

class SafetyManager {
public:
    struct Config {
        float leak_debounce_s = config::LEAK_DEBOUNCE_S;
        float driver_fault_debounce_s = config::DRIVER_FAULT_DEBOUNCE_S;
        float stall_confirm_s = config::STALL_CONFIRM_S;
        float bms_temp_confirm_s = config::BMS_TEMP_CONFIRM_S;
    };

    struct Inputs {
        bool leak_raw = false;

        bool driver_fault_pitch_raw = false;
        bool driver_fault_roll_raw = false;

        bool homing_failed_pitch = false;
        bool homing_failed_roll = false;

        // Expected to come from axis controllers.
        bool stall_pitch_raw = false;
        bool stall_roll_raw = false;

        bool cmd_timeout_small = false;
        bool cmd_timeout_large = false;

        bool bms_timeout_small = false;
        bool bms_timeout_large = false;
        bool bms_temp_high = false;
        bool bms_overcurrent = false;
        bool bms_switch_fault = false;
        bool bms_low_voltage = false;
        bool bms_high_voltage = false;

        bool tof_valid = true;
    };

    struct Output {
        bool hard_fault = false;
        bool soft_fault = false;

        uint8_t hard_fault_a = 0;
        uint8_t hard_fault_b = 0;
        uint8_t soft_fault_bits = 0;

        uint8_t first_hard_fault = 0;
    };

    void configure(const Config& cfg);
    void reset();

    Output update(const Inputs& in, float dt_s);

private:
    enum FirstHardFaultCode : uint8_t {
        FIRST_NONE = 0,
        FIRST_LEAK = 1,
        FIRST_DRIVER_P = 2,
        FIRST_DRIVER_R = 3,
        FIRST_CMD_TO_L = 4,
        FIRST_HOMING_P = 5,
        FIRST_HOMING_R = 6,
        FIRST_BMS_TEMP = 7,
        FIRST_BMS_TO_L = 8,
        FIRST_BMS_OC = 9,
        FIRST_BMS_SWITCH = 10,
        FIRST_STALL_P = 11,
        FIRST_STALL_R = 12
    };

    static bool debounceSignal(bool raw, float& timer_s, float confirm_s, float dt_s);
    void latchFirstHardFault(uint8_t code);

    Config cfg_{};

    bool hard_fault_latched_ = false;

    uint8_t hard_fault_a_ = 0;
    uint8_t hard_fault_b_ = 0;
    uint8_t soft_fault_bits_ = 0;
    uint8_t first_hard_fault_ = 0;

    float stall_pitch_timer_s_ = 0.0f;
    float stall_roll_timer_s_ = 0.0f;
    float bms_temp_timer_s_ = 0.0f;
    float leak_timer_s_ = 0.0f;
    float driver_fault_pitch_timer_s_ = 0.0f;
    float driver_fault_roll_timer_s_ = 0.0f;
};


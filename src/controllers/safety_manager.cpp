#include "controllers/safety_manager.hpp"

#include "protocols/can_messages.hpp"

bool SafetyManager::debounceSignal(bool raw, float& timer_s, float confirm_s, float dt_s)
{
    if (!raw) {
        timer_s = 0.0f;
        return false;
    }

    if (confirm_s <= 0.0f) {
        return true;
    }

    timer_s += dt_s;
    constexpr float kTol = 1e-6f;
    return timer_s + kTol >= confirm_s;
}

void SafetyManager::configure(const Config& cfg)
{
    cfg_ = cfg;

    if (cfg_.leak_debounce_s < 0.0f) {
        cfg_.leak_debounce_s = 0.0f;
    }
    if (cfg_.driver_fault_debounce_s < 0.0f) {
        cfg_.driver_fault_debounce_s = 0.0f;
    }
    if (cfg_.stall_confirm_s < 0.0f) {
        cfg_.stall_confirm_s = 0.0f;
    }
    if (cfg_.bms_temp_confirm_s < 0.0f) {
        cfg_.bms_temp_confirm_s = 0.0f;
    }
}

void SafetyManager::reset()
{
    hard_fault_latched_ = false;

    hard_fault_a_ = 0;
    hard_fault_b_ = 0;
    soft_fault_bits_ = 0;
    first_hard_fault_ = FIRST_NONE;

    stall_pitch_timer_s_ = 0.0f;
    stall_roll_timer_s_ = 0.0f;
    bms_temp_timer_s_ = 0.0f;
    leak_timer_s_ = 0.0f;
    driver_fault_pitch_timer_s_ = 0.0f;
    driver_fault_roll_timer_s_ = 0.0f;
}

void SafetyManager::latchFirstHardFault(uint8_t code)
{
    if (!hard_fault_latched_ && first_hard_fault_ == FIRST_NONE) {
        first_hard_fault_ = code;
    }
}

SafetyManager::Output SafetyManager::update(const Inputs& in, float dt_s)
{
    Output out{};

    if (dt_s <= 0.0f) {
        out.hard_fault = hard_fault_latched_;
        out.soft_fault = (soft_fault_bits_ != 0);
        out.hard_fault_a = hard_fault_a_;
        out.hard_fault_b = hard_fault_b_;
        out.soft_fault_bits = soft_fault_bits_;
        out.first_hard_fault = first_hard_fault_;
        return out;
    }

    const bool leak = debounceSignal(in.leak_raw, leak_timer_s_, cfg_.leak_debounce_s, dt_s);
    const bool driver_pitch = debounceSignal(
        in.driver_fault_pitch_raw,
        driver_fault_pitch_timer_s_,
        cfg_.driver_fault_debounce_s,
        dt_s);
    const bool driver_roll = debounceSignal(
        in.driver_fault_roll_raw,
        driver_fault_roll_timer_s_,
        cfg_.driver_fault_debounce_s,
        dt_s);
    const bool stall_pitch = debounceSignal(
        in.stall_pitch_raw,
        stall_pitch_timer_s_,
        cfg_.stall_confirm_s,
        dt_s);
    const bool stall_roll = debounceSignal(
        in.stall_roll_raw,
        stall_roll_timer_s_,
        cfg_.stall_confirm_s,
        dt_s);
    const bool bms_temp_high = debounceSignal(
        in.bms_temp_high,
        bms_temp_timer_s_,
        cfg_.bms_temp_confirm_s,
        dt_s);

    uint8_t soft = 0;
    if (in.cmd_timeout_small) {
        soft |= can::FAULT_CMD_TO;
    }
    if (!in.tof_valid) {
        soft |= can::FAULT_TOF_INV;
    }
    if (in.bms_timeout_small) {
        soft |= can::FAULT_BMS_TO;
    }
    if (in.bms_low_voltage) {
        soft |= can::FAULT_BMS_LOW;
    }
    if (in.bms_high_voltage) {
        soft |= can::FAULT_BMS_HIGH;
    }
    soft_fault_bits_ = soft;

    uint8_t hardA = 0;
    uint8_t hardB = 0;

    if (leak) {
        hardA |= can::FAULT_LEAK;
        latchFirstHardFault(FIRST_LEAK);
    }

    if (driver_pitch) {
        hardA |= can::FAULT_DRIVER_P;
        latchFirstHardFault(FIRST_DRIVER_P);
    }

    if (driver_roll) {
        hardA |= can::FAULT_DRIVER_R;
        latchFirstHardFault(FIRST_DRIVER_R);
    }

    if (in.cmd_timeout_large) {
        hardA |= can::FAULT_CMD_TO_L;
        latchFirstHardFault(FIRST_CMD_TO_L);
    }

    if (in.homing_failed_pitch) {
        hardA |= can::FAULT_HOMING_FAILED_P;
        latchFirstHardFault(FIRST_HOMING_P);
    }

    if (in.homing_failed_roll) {
        hardA |= can::FAULT_HOMING_FAILED_R;
        latchFirstHardFault(FIRST_HOMING_R);
    }

    if (bms_temp_high) {
        hardA |= can::FAULT_BMS_TEMP;
        latchFirstHardFault(FIRST_BMS_TEMP);
    }

    if (in.bms_timeout_large) {
        hardA |= can::FAULT_BMS_TO_L;
        latchFirstHardFault(FIRST_BMS_TO_L);
    }

    if (in.bms_overcurrent) {
        hardB |= can::FAULT_BMS_OC;
        latchFirstHardFault(FIRST_BMS_OC);
    }

    if (in.bms_switch_fault) {
        hardB |= can::FAULT_BMS_SWITCH;
        latchFirstHardFault(FIRST_BMS_SWITCH);
    }

    if (stall_pitch) {
        hardB |= can::FAULT_STALL_P;
        latchFirstHardFault(FIRST_STALL_P);
    }

    if (stall_roll) {
        hardB |= can::FAULT_STALL_R;
        latchFirstHardFault(FIRST_STALL_R);
    }

    if (hardA != 0 || hardB != 0) {
        hard_fault_latched_ = true;
    }

    if (hard_fault_latched_) {
        hard_fault_a_ |= hardA;
        hard_fault_b_ |= hardB;
    }

    out.hard_fault = hard_fault_latched_;
    out.soft_fault = (soft_fault_bits_ != 0);
    out.hard_fault_a = hard_fault_a_;
    out.hard_fault_b = hard_fault_b_;
    out.soft_fault_bits = soft_fault_bits_;
    out.first_hard_fault = first_hard_fault_;

    return out;
}

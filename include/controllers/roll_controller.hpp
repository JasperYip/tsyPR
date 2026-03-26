#pragma once

#include <stdint.h>

#include "config/constants.hpp"

class RollController {
public:
    enum class State : uint8_t {
        UNHOMED = 0,
        HOMING = 1,
        RUN = 2,
        HOLD = 3,
        FAULT = 4
    };

    struct Config {
        float step_deg = config::ROLL_STEP_DEG;

        float angle_min_deg = -25.0f;
        float angle_max_deg = 25.0f;

        float run_speed_deg_s = 20.0f;
        float homing_speed_deg_s = 10.0f;

        float homing_timeout_s = config::HOMING_TIMEOUT_S;
        float min_hall_band_deg = 1.0f;
    };

    struct Inputs {
        bool hard_fault = false;

        bool cmd_enable = false;
        bool cmd_home = false;

        float target_deg = 0.0f;

        bool hall_active = false;
    };

    struct Output {
        bool enable_motor = false;

        // CW is positive roll direction.
        bool dir_cw = true;
        uint32_t step_count = 0;
        bool moving = false;

        State state = State::UNHOMED;
        bool homed = false;
        bool homing_failed = false;
        bool stall_raw = false;

        float angle_deg = 0.0f;
        float target_deg = 0.0f;
        bool hall_active = false;
    };

    RollController() = default;
    explicit RollController(const Config& cfg);

    void configure(const Config& cfg);
    void reset();

    Output update(const Inputs& in, float dt_s);

    State state() const { return state_; }
    bool homed() const { return homed_; }
    bool homingFailed() const { return homing_failed_latched_; }
    float angleDeg() const;

private:
    enum class HomingPhase : uint8_t {
        IDLE = 0,
        SEEK_CW_CLEAR = 1,
        SEEK_CCW_ENTER = 2,
        SEEK_CCW_CLEAR = 3,
        MOVE_TO_CENTER = 4
    };

    static float clampf(float v, float min_v, float max_v);

    int32_t degToSteps(float deg) const;
    float stepsToDeg(int32_t steps) const;

    uint32_t stepBudget(float speed_deg_s, float dt_s);
    void applyMotion(bool dir_cw, uint32_t steps);
    void startHoming();
    void failHoming();

    Config cfg_{};

    State state_ = State::UNHOMED;
    HomingPhase homing_phase_ = HomingPhase::IDLE;

    bool homed_ = false;
    bool homing_failed_latched_ = false;
    bool last_cmd_home_ = false;

    float step_residual_ = 0.0f;
    float homing_timer_s_ = 0.0f;

    int32_t pos_steps_ = 0;
    int32_t target_steps_ = 0;

    int32_t hall_enter_steps_ = 0;
    int32_t hall_exit_steps_ = 0;
    int32_t center_target_steps_ = 0;
};


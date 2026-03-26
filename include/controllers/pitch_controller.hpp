#pragma once

#include <stdint.h>

#include "config/constants.hpp"

class PitchController {
public:
    enum class State : uint8_t {
        UNHOMED = 0,
        HOMING = 1,
        RUN = 2,
        HOLD = 3,
        FAULT = 4
    };

    struct Config {
        float step_mm = config::PITCH_STEP_MM;

        float travel_min_mm = -60.0f;
        float travel_max_mm = 60.0f;

        float run_speed_mm_s = 20.0f;
        float homing_speed_mm_s = 10.0f;

        float homing_timeout_s = config::HOMING_TIMEOUT_S;
        float min_homing_span_mm = 5.0f;
    };

    struct Inputs {
        bool hard_fault = false;

        bool cmd_enable = false;
        bool cmd_home = false;

        float target_mm = 0.0f;

        bool lim_fwd_pressed = false;
        bool lim_rev_pressed = false;
    };

    struct Output {
        bool enable_motor = false;

        // Forward is positive pitch travel direction.
        bool dir_forward = true;
        uint32_t step_count = 0;
        bool moving = false;

        State state = State::UNHOMED;
        bool homed = false;
        bool homing_failed = false;
        bool stall_raw = false;

        float pos_mm = 0.0f;
        float target_mm = 0.0f;

        bool at_fwd_limit = false;
        bool at_rev_limit = false;
    };

    PitchController() = default;
    explicit PitchController(const Config& cfg);

    void configure(const Config& cfg);
    void reset();

    Output update(const Inputs& in, float dt_s);

    State state() const { return state_; }
    bool homed() const { return homed_; }
    bool homingFailed() const { return homing_failed_latched_; }
    float positionMm() const;

private:
    enum class HomingPhase : uint8_t {
        IDLE = 0,
        SEEK_FWD = 1,
        SEEK_REV = 2,
        MOVE_TO_CENTER = 3
    };

    static float clampf(float v, float min_v, float max_v);

    int32_t mmToSteps(float mm) const;
    float stepsToMm(int32_t steps) const;

    uint32_t stepBudget(float speed_mm_s, float dt_s);
    void applyMotion(bool dir_forward, uint32_t steps);
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

    int32_t fwd_hit_steps_ = 0;
    int32_t rev_hit_steps_ = 0;
    int32_t center_target_steps_ = 0;
};


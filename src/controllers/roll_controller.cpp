#include "controllers/roll_controller.hpp"

#include <cmath>

RollController::RollController(const Config& cfg)
{
    configure(cfg);
    reset();
}

void RollController::configure(const Config& cfg)
{
    cfg_ = cfg;

    if (cfg_.step_deg <= 0.0f) {
        cfg_.step_deg = config::ROLL_STEP_DEG;
    }
    if (cfg_.angle_max_deg < cfg_.angle_min_deg) {
        const float t = cfg_.angle_min_deg;
        cfg_.angle_min_deg = cfg_.angle_max_deg;
        cfg_.angle_max_deg = t;
    }
    if (cfg_.run_speed_deg_s < 0.0f) {
        cfg_.run_speed_deg_s = 0.0f;
    }
    if (cfg_.homing_speed_deg_s < 0.0f) {
        cfg_.homing_speed_deg_s = 0.0f;
    }
    if (cfg_.homing_timeout_s < 0.0f) {
        cfg_.homing_timeout_s = 0.0f;
    }
    if (cfg_.min_hall_band_deg < 0.0f) {
        cfg_.min_hall_band_deg = 0.0f;
    }
}

void RollController::reset()
{
    state_ = State::UNHOMED;
    homing_phase_ = HomingPhase::IDLE;

    homed_ = false;
    homing_failed_latched_ = false;
    last_cmd_home_ = false;

    step_residual_ = 0.0f;
    homing_timer_s_ = 0.0f;

    pos_steps_ = 0;
    target_steps_ = 0;

    hall_enter_steps_ = 0;
    hall_exit_steps_ = 0;
    center_target_steps_ = 0;
}

float RollController::angleDeg() const
{
    return stepsToDeg(pos_steps_);
}

float RollController::clampf(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

int32_t RollController::degToSteps(float deg) const
{
    return static_cast<int32_t>(std::lround(deg / cfg_.step_deg));
}

float RollController::stepsToDeg(int32_t steps) const
{
    return static_cast<float>(steps) * cfg_.step_deg;
}

uint32_t RollController::stepBudget(float speed_deg_s, float dt_s)
{
    if (speed_deg_s <= 0.0f || dt_s <= 0.0f) {
        return 0;
    }

    const float steps_f = (speed_deg_s * dt_s / cfg_.step_deg) + step_residual_;
    const uint32_t steps = (steps_f > 0.0f) ? static_cast<uint32_t>(steps_f) : 0u;

    step_residual_ = steps_f - static_cast<float>(steps);
    if (step_residual_ < 0.0f) {
        step_residual_ = 0.0f;
    }
    return steps;
}

void RollController::applyMotion(bool dir_cw, uint32_t steps)
{
    if (steps == 0) {
        return;
    }

    const int32_t delta = static_cast<int32_t>(steps);
    pos_steps_ += dir_cw ? delta : -delta;
}

void RollController::startHoming()
{
    state_ = State::HOMING;
    homing_phase_ = HomingPhase::SEEK_CW_CLEAR;
    homing_timer_s_ = 0.0f;
    step_residual_ = 0.0f;

    homed_ = false;
    homing_failed_latched_ = false;

    hall_enter_steps_ = pos_steps_;
    hall_exit_steps_ = pos_steps_;
    center_target_steps_ = pos_steps_;
}

void RollController::failHoming()
{
    homing_failed_latched_ = true;
    homed_ = false;
    state_ = State::UNHOMED;
    homing_phase_ = HomingPhase::IDLE;
}

RollController::Output RollController::update(const Inputs& in, float dt_s)
{
    Output out{};

    const bool home_rise = in.cmd_home && !last_cmd_home_;
    last_cmd_home_ = in.cmd_home;

    if (in.hard_fault) {
        state_ = State::FAULT;
    }

    if (state_ == State::FAULT) {
        out.state = state_;
        out.homed = homed_;
        out.homing_failed = homing_failed_latched_;
        out.angle_deg = stepsToDeg(pos_steps_);
        out.target_deg = clampf(in.target_deg, cfg_.angle_min_deg, cfg_.angle_max_deg);
        out.hall_active = in.hall_active;
        return out;
    }

    if (home_rise) {
        startHoming();
    }

    if (state_ == State::UNHOMED && in.cmd_enable && homed_) {
        state_ = State::RUN;
    } else if (state_ == State::RUN && !in.cmd_enable) {
        state_ = State::HOLD;
    } else if (state_ == State::HOLD && in.cmd_enable && homed_) {
        state_ = State::RUN;
    }

    out.target_deg = clampf(in.target_deg, cfg_.angle_min_deg, cfg_.angle_max_deg);
    target_steps_ = degToSteps(out.target_deg);
    out.hall_active = in.hall_active;

    if (state_ == State::HOMING) {
        if (dt_s > 0.0f) {
            homing_timer_s_ += dt_s;
        }
        if (cfg_.homing_timeout_s > 0.0f && homing_timer_s_ > cfg_.homing_timeout_s) {
            failHoming();
        }
    }

    switch (state_) {
    case State::UNHOMED:
        out.enable_motor = false;
        break;

    case State::HOLD:
        out.enable_motor = false;
        break;

    case State::HOMING: {
        out.enable_motor = true;

        if (state_ != State::HOMING) {
            break;
        }

        const uint32_t budget = stepBudget(cfg_.homing_speed_deg_s, dt_s);

        if (homing_phase_ == HomingPhase::SEEK_CW_CLEAR) {
            if (!in.hall_active) {
                homing_phase_ = HomingPhase::SEEK_CCW_ENTER;
            } else if (budget > 0) {
                out.dir_cw = true;
                out.step_count = budget;
                out.moving = true;
                applyMotion(true, budget);
            }
        } else if (homing_phase_ == HomingPhase::SEEK_CCW_ENTER) {
            if (in.hall_active) {
                hall_enter_steps_ = pos_steps_;
                homing_phase_ = HomingPhase::SEEK_CCW_CLEAR;
            } else if (budget > 0) {
                out.dir_cw = false;
                out.step_count = budget;
                out.moving = true;
                applyMotion(false, budget);
            }
        } else if (homing_phase_ == HomingPhase::SEEK_CCW_CLEAR) {
            if (!in.hall_active) {
                hall_exit_steps_ = pos_steps_;
                const int32_t span_steps = hall_enter_steps_ - hall_exit_steps_;
                const int32_t min_span_steps = degToSteps(cfg_.min_hall_band_deg);

                if (span_steps <= min_span_steps) {
                    failHoming();
                    break;
                }

                center_target_steps_ = hall_exit_steps_ + (span_steps / 2);
                homing_phase_ = HomingPhase::MOVE_TO_CENTER;
            } else if (budget > 0) {
                out.dir_cw = false;
                out.step_count = budget;
                out.moving = true;
                applyMotion(false, budget);
            }
        } else if (homing_phase_ == HomingPhase::MOVE_TO_CENTER) {
            const int32_t delta = center_target_steps_ - pos_steps_;
            if (delta == 0) {
                pos_steps_ = 0;
                target_steps_ = 0;
                homed_ = true;
                homing_phase_ = HomingPhase::IDLE;
                state_ = in.cmd_enable ? State::RUN : State::HOLD;
            } else if (budget > 0) {
                const bool dir_cw = (delta > 0);
                uint32_t steps = static_cast<uint32_t>((delta > 0) ? delta : -delta);
                if (steps > budget) {
                    steps = budget;
                }

                out.dir_cw = dir_cw;
                out.step_count = steps;
                out.moving = (steps > 0);
                applyMotion(dir_cw, steps);
            }
        }
    } break;

    case State::RUN: {
        out.enable_motor = true;

        const int32_t delta = target_steps_ - pos_steps_;
        if (delta == 0) {
            break;
        }

        const bool dir_cw = (delta > 0);

        uint32_t steps = stepBudget(cfg_.run_speed_deg_s, dt_s);
        if (steps == 0) {
            break;
        }

        uint32_t abs_delta = static_cast<uint32_t>((delta > 0) ? delta : -delta);
        if (steps > abs_delta) {
            steps = abs_delta;
        }

        const int32_t min_steps = degToSteps(cfg_.angle_min_deg);
        const int32_t max_steps = degToSteps(cfg_.angle_max_deg);

        if (dir_cw) {
            const int32_t room = max_steps - pos_steps_;
            if (room <= 0) {
                steps = 0;
            } else if (steps > static_cast<uint32_t>(room)) {
                steps = static_cast<uint32_t>(room);
            }
        } else {
            const int32_t room = pos_steps_ - min_steps;
            if (room <= 0) {
                steps = 0;
            } else if (steps > static_cast<uint32_t>(room)) {
                steps = static_cast<uint32_t>(room);
            }
        }

        out.dir_cw = dir_cw;
        out.step_count = steps;
        out.moving = (steps > 0);
        applyMotion(dir_cw, steps);
    } break;

    case State::FAULT:
    default:
        break;
    }

    out.state = state_;
    out.homed = homed_;
    out.homing_failed = homing_failed_latched_;
    out.stall_raw = false;
    out.angle_deg = stepsToDeg(pos_steps_);
    out.target_deg = stepsToDeg(target_steps_);
    out.hall_active = in.hall_active;

    if (state_ == State::UNHOMED || state_ == State::HOLD || state_ == State::FAULT) {
        out.enable_motor = false;
        out.moving = false;
        out.step_count = 0;
    }

    return out;
}

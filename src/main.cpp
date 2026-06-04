#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "config/constants.hpp"
#include "config/pin_map.hpp"
#include "utils/logger.hpp"

#include "drivers/bms.hpp"
#include "drivers/can_bus.hpp"
#include "drivers/hall_sensor.hpp"
#include "drivers/leak_sensor.hpp"
#include "drivers/push_switch.hpp"
#include "drivers/stepper_driver.hpp"
#include "drivers/tof_sensor.hpp"

#include "controllers/bms_manager.hpp"
#include "controllers/safety_manager.hpp"

#include "protocols/can_messages.hpp"

// ----------------------------------------------------------------
// Hardware configuration
// ----------------------------------------------------------------
// Set false to run without BMS hardware connected.
constexpr bool USE_BMS = true;

// Set false to mask the pitch driver FLT pin from safety faults.
// RST-controlled startup now prevents boot-noise false faults — keep true.
constexpr bool USE_PITCH_FLT = true;

// Disable stepper driver when not moving to prevent idle heat buildup.
// The leadscrew is self-locking so position is held without holding current.
// Driver re-enables automatically a few ms before each move starts.
constexpr bool     AUTO_DISABLE_IDLE = true;
constexpr uint32_t IDLE_DISABLE_MS   = 1000;  // ms after last move before disabling

// Set false to disable ToF sensor (VL6180X on I2C1).
// When true: tof_mm in STATUS_CONTROL carries live distance;
//            jogPitch() uses ToF to detect stall and auto-reduce speed.
constexpr bool     USE_TOF        = true;
constexpr uint16_t TOF_NEUTRAL_MM = 65;   // expected reading at home/neutral position
constexpr uint32_t TOF_POLL_MS    = 50;   // background poll interval (ms)
// Stall detection: if ToF moves less than this per 1mm of commanded jog → stall
// EMA smoothing on raw ToF readings — reduces sensor jitter on the reported value.
// alpha=0.25: new reading weighted at 25%, history at 75%. Lower = smoother but laggier.
constexpr float    TOF_EMA_ALPHA          = 0.25f;
// Stall detection threshold — must be > typical noise floor (~2mm for VL6180X)
constexpr uint16_t TOF_STALL_THRESHOLD_MM = 2;
// Require this many consecutive stall chunks before reducing speed (filters noise spikes)
constexpr uint8_t  TOF_STALL_CONFIRM      = 2;
// Auto torque: each confirmed stall reduces pitchJogSpeed by this fraction
// 0.70 = −30% per stall event — aggressive drop gets to max torque faster
constexpr float    TOF_STALL_SPEED_FACTOR = 0.70f;
constexpr float    TOF_STALL_SPEED_MIN    = 800.0f;  // floor = PITCH_RAMP_START_SPD — never go below ramp entry speed
// Jiggle recovery: when at min speed and still stalled, reverse this far then retry
constexpr float    TOF_JIGGLE_MM          = 2.0f;    // mm to reverse per jiggle
constexpr uint8_t  TOF_JIGGLE_MAX         = 3;       // max jiggle attempts before giving up

// Set false to bypass the roll axis entirely.
// homeRoll() will immediately return success so the Pi sees roll as homed.
// All roll movement commands are silently skipped.
// Useful when the roll motor is not physically wired.
constexpr bool USE_ROLL = true;

// ----------------------------------------------------------------
// Direction conventions — flip if a motor runs backwards
// ----------------------------------------------------------------
static constexpr StepperDriver::Direction PITCH_FORWARD_DIR = StepperDriver::Direction::CCW;
static constexpr StepperDriver::Direction ROLL_CW_DIR       = StepperDriver::Direction::CW;

// ----------------------------------------------------------------
// Motion parameters
// ----------------------------------------------------------------
// Pitch runs slower than roll — lower speed stays in the high-torque
// region of the stepper curve; going too slow causes DRV8825 OTP (overtemp).
// Speeds are in microsteps/s (1/16 microstepping — M2 pin soldered HIGH on TB67S581FNG).
// Physical mm/s is unchanged: PITCH_STEP_MM is now 0.000625mm so ×16 steps/s = same mm/s.
// e.g. 6400 microsteps/s × 0.000625mm = 4.0 mm/s  (was 400 steps/s × 0.01mm = 4.0 mm/s ✓)
static constexpr float    PITCH_HOME_SPEED     = 16000.0f; // microsteps/s during homing  (10.0 mm/s @ 1/16)
static constexpr float    ROLL_HOME_SPEED      = 400.0f;   // pps during homing (NFP-GW4632-25BY 337:1)
static constexpr float    PITCH_RUN_SPEED      = 16000.0f; // microsteps/s for CAN setpoint moves (10.0 mm/s @ 1/16)
static constexpr float    ROLL_RUN_SPEED       = 700.0f;   // pps for CAN setpoint moves (31.5°/s @ 337:1)
static constexpr uint32_t PITCH_HOME_MAX_STEPS = 480000;   // same physical travel × 16 (was 30000)
// Trapezoidal ramp — avoids resonance band on startup and prevents step loss on direction change.
// min(accel_curve, decel_curve) approach: short moves automatically form a triangle profile.
static constexpr float    PITCH_RAMP_START_SPD = 800.0f;   // microsteps/s — ramp entry/exit speed
static constexpr float    PITCH_RAMP_ACCEL     = 32000.0f; // microsteps/s² — ramp in ~0.5mm
static constexpr uint32_t ROLL_HOME_MAX_STEPS  = 15000;

// Jog speeds — runtime adjustable via 'pspd' / 'rspd' commands.
// Lower pitch jog speed = more torque (less back-EMF).
// Default 300 steps/s gives good torque without OTP risk.
static float pitchJogSpeed = 9600.0f;  // microsteps/s jog speed (6.0 mm/s @ 1/16)
static float rollJogSpeed  = 500.0f;   // pps jog speed

// ----------------------------------------------------------------
// CAN
// ----------------------------------------------------------------
static constexpr uint8_t  CAN_NODE_ID                = config::NODE_ID_PR;  // NODE 3
static constexpr uint32_t CAN_TX_CONTROL_INTERVAL_MS = 20;   // 50 Hz
static constexpr uint32_t CAN_TX_FAULT_INTERVAL_MS   = 500;  //  2 Hz
static constexpr uint32_t CAN_TX_BMS_INTERVAL_MS     = 500;  //  2 Hz

// ----------------------------------------------------------------
// Drivers
// ----------------------------------------------------------------
// Pitch motor on pitch driver board (pins 2/3/4/5).
// PIN_PITCH_SLP_RST (pin 6) tied to SLP+RST — HIGH = driver awake, LOW = sleep+reset.
// TB67S581FNG on Pololu carrier: en_active_low = true (confirmed working)
static StepperDriver pitch({
    .pin_step       = PIN_PITCH_STEP,   // pin 3
    .pin_dir        = PIN_PITCH_DIR,    // pin 2
    .pin_en         = PIN_PITCH_ENA,    // pin 4
    .pin_flt        = PIN_PITCH_FLT,    // pin 5
    .step_pulse_us  = 5.0f,
    .steps_per_sec  = PITCH_RUN_SPEED,
    .flt_active_low = true,
    .en_active_low  = true
});

static StepperDriver roll({
    .pin_step       = PIN_ROLL_STEP,
    .pin_dir        = PIN_ROLL_DIR,
    .pin_en         = PIN_ROLL_ENA,
    .pin_flt        = PIN_ROLL_FLT,
    .step_pulse_us  = 5.0f,
    .steps_per_sec  = ROLL_RUN_SPEED,
    .flt_active_low = true,
    .en_active_low  = true
});

static PushSwitch pitchFwdLimit({ .pin = PIN_FWD_SWH,   .active_low = true, .use_pullup = true });
static PushSwitch pitchRevLimit({ .pin = PIN_BWD_SWH,   .active_low = true, .use_pullup = true });
static HallSensor rollHall     ({ .pin = PIN_HALL_SENS, .active_low = true, .use_pullup = true });
static LeakSensor leak(PIN_LEAK, true);  // active HIGH

static ToFSensor  tof(PIN_TOF_SDA, PIN_TOF_SCL);
static uint16_t   tofMm        = 0;     // EMA-filtered reading in mm
static bool       tofValid     = false;
static float      tofEma       = 0.0f;  // internal EMA accumulator
static uint32_t   lastTofPoll  = 0;
// ToF direction is fixed by mounting: sensor faces the forward plate, so
// moving FORWARD closes the gap → ToF reading decreases.
// Therefore: tofMm < neutral  → in forward half  → FWD limit is nearer
//            tofMm > neutral  → in backward half → BWD limit is nearer

static CanBus     canBus;
static BmsUart    bmsUart({ .serial = &Serial2, .baud = BMS_BAUD });
static BmsManager bmsManager;
static SafetyManager safety;
static SafetyManager::Output lastSafety{};

// ----------------------------------------------------------------
// Axis state
// ----------------------------------------------------------------
static int32_t pitchSteps        = 0;
static int32_t rollSteps         = 0;
static bool    pitchHomed        = false;
static bool    rollHomed         = false;
static bool    pitchHomingFailed = false;
static bool    rollHomingFailed  = false;
static bool    motorsEnabled     = false;
static uint32_t lastPitchMoveMs = 0;    // updated after every pitch step — drives pitch idle-disable
static uint32_t lastRollMoveMs  = 0;    // updated after every roll step  — drives roll  idle-disable
static bool    pitchIdleOff     = false; // true when pitch driver was disabled by idle timer
static bool    rollIdleOff      = false; // true when roll driver was disabled by idle timer

// ----------------------------------------------------------------
// Control mode
// ----------------------------------------------------------------
enum class Mode { CAN, OVERRIDE, HOMING, NEUTRAL };
static Mode mode             = Mode::CAN;
static Mode homingReturnMode = Mode::CAN;

// Pending setpoint — written by CAN RX, consumed by loop dispatch
static float targetPitchMm      = 0.0f;
static float targetRollDeg      = 0.0f;
static bool  newSetpointPending = false;

// ----------------------------------------------------------------
// CAN bookkeeping
// ----------------------------------------------------------------
static uint32_t lastCanRx     = millis();  // initialised at boot — timeout only starts after first CMD_SETPOINT
static bool     canRxEver     = false;
static uint8_t  canSeqControl = 0;
static uint8_t  canSeqFault   = 0;
static uint8_t  canSeqBms     = 0;
static uint8_t  canSeqBmsMore = 0;
static uint32_t lastTxControl = 0;
static uint32_t lastTxFault   = 0;
static uint32_t lastTxBms     = 0;

// De-dup cache: only print CAN frames whose payload changed (ignoring seq byte)
struct FrameCache { bool valid; CanFrame frame; };
static FrameCache lastFrames[2048];

// ----------------------------------------------------------------
// Serial
// ----------------------------------------------------------------
static String   cmdBuffer;
static bool     debugPrint = false;
static uint32_t lastPrint  = 0;

// ----------------------------------------------------------------
// LED heartbeat
// ----------------------------------------------------------------
static uint32_t lastLedToggle = 0;
static bool     ledOn         = false;

// ----------------------------------------------------------------
// Fault injection flags (debug / test only)
// ----------------------------------------------------------------
static bool inj_leak        = false;
static bool inj_driver_p    = false;
static bool inj_driver_r    = false;
static bool inj_cmd_s       = false;  // soft CAN timeout
static bool inj_cmd_h       = false;  // hard CAN timeout
static bool inj_bms_to_s    = false;
static bool inj_bms_to_h    = false;
static bool inj_bms_temp    = false;
static bool inj_bms_oc      = false;
static bool inj_bms_sw      = false;
static bool inj_bms_low     = false;
static bool inj_bms_high    = false;
static bool inj_home_fail_p = false;
static bool inj_home_fail_r = false;

// ================================================================
// Helpers
// ================================================================

static StepperDriver::Direction opposite(StepperDriver::Direction d) {
    return d == StepperDriver::Direction::CW
        ? StepperDriver::Direction::CCW
        : StepperDriver::Direction::CW;
}

static int32_t pitchMmToSteps(float mm) {
    return static_cast<int32_t>(lroundf(mm / config::PITCH_STEP_MM));
}
static int32_t rollDegToSteps(float deg) {
    return static_cast<int32_t>(lroundf(deg / config::ROLL_STEP_DEG));
}
static float   pitchStepsToMm(int32_t s)  { return s * config::PITCH_STEP_MM; }
static float   rollStepsToDeg(int32_t s)  { return s * config::ROLL_STEP_DEG; }
static uint32_t absI32(int32_t v)          { return v >= 0 ? (uint32_t)v : (uint32_t)(-v); }

// Re-enable any idle-disabled drivers and reset the idle timer.
// Call before any direct pitch.step() usage that bypasses movePitchDelta.
static void wakeDrivers() {
    const uint32_t now = millis();
    if (AUTO_DISABLE_IDLE) {
        if (pitchIdleOff) { pitch.enable(); pitchIdleOff = false; delay(10); }
        if (USE_ROLL && rollIdleOff) { roll.enable(); rollIdleOff = false; delay(10); }
    }
    lastPitchMoveMs = now;
    lastRollMoveMs  = now;
}

// Any raw signal that demands immediate move abort
static bool shouldAbortMove() {
    return (leak.isLeak() || inj_leak)
        || (USE_PITCH_FLT && pitch.faultActive())
        || (USE_ROLL && roll.faultActive());
}

// ----------------------------------------------------------------
// Driver reset helpers
// ----------------------------------------------------------------
// TB67S581FNG latches a fault (OCP/OTP) on the FLT pin.
// Driving RST LOW clears the latch — more reliable than EN toggle.
// SLP+RST are tied together on pin 6: LOW = sleep+reset, HIGH = run.
static void resetPitchDriver() {
    digitalWrite(PIN_PITCH_SLP_RST, LOW);   // sleep + reset → fault latch cleared
    delay(500);                              // thermal cooldown if OTP was the cause
    digitalWrite(PIN_PITCH_SLP_RST, HIGH);  // wake driver
    delay(10);
    if (USE_PITCH_FLT && pitch.faultActive()) {
        Serial.println("[drv] WARN: pitch fault still active after reset");
    }
}

static void resetRollDriver() {
    if (!USE_ROLL) return;
    roll.disable();
    delay(100);
    roll.enable();
    delay(10);
    if (roll.faultActive()) {
        Serial.println("[drv] WARN: roll fault still active after reset");
    }
}

// ----------------------------------------------------------------
// Primitive moves (used by homing and setpoint execution)
// ----------------------------------------------------------------

// Move pitch by |delta| steps in the correct direction.
// Checks FWD/BWD limit switches and fault/leak every step.
//
// Mid-move OCP recovery: when the driver asserts FLT (OCP/OTP) during a move,
// the driver is RST-reset and the speed is halved before retrying the remaining
// steps.  This handles the common case where a loaded jog at moderate speed
// triggers OCP — the move completes at a safer speed without requiring a manual
// 'recover' + retry.  Up to MAX_FAULT_RETRIES retries; position counter is NOT
// corrupted because the fault is detected before the failed step is taken.
// Only if retries are exhausted does it abort and mark position as lost.
static void sendStatusControl();  // forward declaration — defined later
static void tickHoming();         // forward declaration — defined later
static void handleCan();          // forward declaration — defined later
static bool movePitchDelta(int32_t delta) {
    if (delta == 0) return true;

    // Wake driver if idle-disabled — give it a few ms to stabilise before stepping
    if (AUTO_DISABLE_IDLE && pitchIdleOff) {
        pitch.enable();
        pitchIdleOff = false;
        delay(10);
    }

    const bool fwd = delta > 0;
    uint32_t remaining = absI32(delta);
    pitch.setDirection(fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));

    uint8_t faultRetries = 0;
    constexpr uint8_t MAX_FAULT_RETRIES = 2;

    // Trapezoidal ramp — avoids resonance at low speed and step loss at direction change.
    // Speed = min(accel_curve, decel_curve), clamped to [RAMP_START, rampTarget].
    // Short moves naturally form a triangle (no flat top) with no extra logic.
    // On OCP retry: rampTarget is halved and the ramp restarts from RAMP_START.
    float rampTarget = pitch.speed();
    uint32_t stepsDone = 0;
    pitch.setSpeed(min(PITCH_RAMP_START_SPD, rampTarget));

    uint8_t jiggleCount = 0;
    uint8_t tofStallRetries = 0;  // separate from faultRetries — OCP and ToF stall are independent
    bool    retarget = false;     // set when a new CAN setpoint arrives mid-move

    // Periodic serial + ToF stall state — checked every 16 steps.
    // Serial progress only printed for moves > 5mm — suppresses noise from
    // short ToF correction steps and other small internal moves.
    // ToF stall gated until motor has travelled 3mm — avoids false triggers
    // during ramp-up where the motor moves <1mm per 100ms check interval.
    const bool printProgress  = (int32_t)absI32(delta) > pitchMmToSteps(5.0f);
    const uint32_t stallGate  = (uint32_t)pitchMmToSteps(3.0f);  // start checking after 3mm
    uint32_t lastPrintMs  = millis();
    uint32_t lastTofMs    = millis();
    uint16_t prevTofRaw   = tofMm;
    uint8_t  tofStallCount = 0;

    while (remaining > 0) {
        // Every 16 steps: serial progress + ToF stall check
        if ((stepsDone & 15) == 0 && stepsDone > 0) {
            const uint32_t now = millis();

            // Process incoming CAN — updates newSetpointPending if Pi sent a new target
            handleCan();
            tickHoming();

            // If a new setpoint arrived in CAN mode, break out so main loop
            // can immediately execute the updated target instead of finishing here.
            if (newSetpointPending && mode == Mode::CAN) {
                retarget = true;
            }

            // Serial progress — every 1s, only for moves > 5mm
            if (printProgress && (now - lastPrintMs >= 1000)) {
                lastPrintMs = now;
                const float posMm = (USE_TOF && tofValid && pitchHomed)
                    ? ((float)TOF_NEUTRAL_MM - tofEma) : pitchStepsToMm(pitchSteps);
                Serial.print("[move] pos="); Serial.print(posMm, 1);
                Serial.print("mm  tof="); Serial.print(tofMm);
                Serial.print("mm  remain="); Serial.print(remaining * config::PITCH_STEP_MM, 1);
                Serial.println("mm");
            }

            // ToF stall check — every 500ms, only after 3mm of travel (past ramp-up).
            // 500ms @ 10mm/s = 5mm expected movement — well above VL6180X 1mm noise floor.
            // Three consecutive no-movement reads = 1.5s of confirmed zero ToF change = real stall.
            if (USE_TOF && tofValid && stepsDone >= stallGate && (now - lastTofMs >= 500)) {
                lastTofMs = now;
                const ToFSensor::Reading r = tof.read();
                if (r.valid) {
                    tofEma = TOF_EMA_ALPHA * r.mm + (1.0f - TOF_EMA_ALPHA) * tofEma;
                    tofMm  = static_cast<uint16_t>(roundf(tofEma));
                    const uint16_t rawDelta = (r.mm > prevTofRaw)
                        ? (r.mm - prevTofRaw) : (prevTofRaw - r.mm);
                    if (rawDelta < 1) {
                        tofStallCount++;
                    } else {
                        tofStallCount = 0;
                    }
                    prevTofRaw = r.mm;

                    if (tofStallCount >= 3 && tofStallRetries < TOF_JIGGLE_MAX) {
                        tofStallRetries++;
                        jiggleCount++;
                        tofStallCount = 0;
                        Serial.print("[move] ToF stall — jiggle ");
                        Serial.print(tofStallRetries); Serial.print("/"); Serial.print(TOF_JIGGLE_MAX);
                        Serial.print("  tof="); Serial.print(tofMm); Serial.println("mm");
                        resetPitchDriver();
                        const int32_t jiggleSteps = pitchMmToSteps(TOF_JIGGLE_MM);
                        pitch.setSpeed(PITCH_RAMP_START_SPD);
                        pitch.setDirection(fwd ? opposite(PITCH_FORWARD_DIR) : PITCH_FORWARD_DIR);
                        for (int32_t j = 0; j < jiggleSteps; j++) pitch.step();
                        pitchSteps += fwd ? -jiggleSteps : jiggleSteps;
                        remaining  += jiggleSteps;
                        pitch.setDirection(fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
                        // No speed reduction — ToF stall is a mechanical engagement issue,
                        // not a torque issue. Keep rampTarget unchanged.
                        pitch.setSpeed(min(PITCH_RAMP_START_SPD, rampTarget));
                        stepsDone = stallGate;  // restart past gate so stall check resumes quickly
                        delay(50);
                        prevTofRaw = tofMm;
                        lastTofMs  = millis();
                        continue;
                    }
                }
            }
        }

        if (retarget) break;  // new CAN setpoint received — exit and let main loop retarget

        if (USE_PITCH_FLT && pitch.faultActive()) {
            if (faultRetries < MAX_FAULT_RETRIES) {
                faultRetries++;
                const float newSpd = max(TOF_STALL_SPEED_MIN, rampTarget * 0.5f);
                Serial.print("[move] pitch OCP — auto-recover ");
                Serial.print(faultRetries); Serial.print("/"); Serial.print(MAX_FAULT_RETRIES);
                Serial.print("  spd "); Serial.print(rampTarget, 0);
                Serial.print("->"); Serial.print(newSpd, 0);
                Serial.print("  remain="); Serial.print(remaining); Serial.println(" steps");
                resetPitchDriver();

                // Jiggle: reverse a short distance to re-engage the leadscrew,
                // then come back — helps when the nut has climbed off the thread.
                if (jiggleCount < TOF_JIGGLE_MAX) {
                    jiggleCount++;
                    const int32_t jiggleSteps = pitchMmToSteps(TOF_JIGGLE_MM);
                    Serial.print("[move] jiggle "); Serial.print(jiggleCount);
                    Serial.print("/"); Serial.print(TOF_JIGGLE_MAX);
                    Serial.print(" ("); Serial.print(TOF_JIGGLE_MM, 1); Serial.println("mm reverse)");
                    pitch.setSpeed(PITCH_RAMP_START_SPD);
                    pitch.setDirection(fwd ? opposite(PITCH_FORWARD_DIR) : PITCH_FORWARD_DIR);
                    for (int32_t j = 0; j < jiggleSteps; j++) pitch.step();
                    pitchSteps += fwd ? -jiggleSteps : jiggleSteps;
                    remaining  += jiggleSteps;  // account for ground lost
                    pitch.setDirection(fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
                    delay(50);
                }

                rampTarget = newSpd;
                pitch.setSpeed(min(PITCH_RAMP_START_SPD, rampTarget));
                if (newSpd < pitchJogSpeed) pitchJogSpeed = newSpd;
                stepsDone = 0;
                delay(50);
                continue;
            }
            Serial.println("[move] ABORT pitch: fault after retries — position lost, rehome required");
            pitchHomed = false;
            return false;
        }
        if (leak.isLeak() || inj_leak) {
            Serial.println("[move] ABORT pitch: leak detected");
            return false;
        }
        if (fwd  && pitchFwdLimit.isPressed()) { Serial.println("[move] STOP pitch: FWD limit hit"); return false; }
        if (!fwd && pitchRevLimit.isPressed()) { Serial.println("[move] STOP pitch: BWD limit hit"); return false; }

        // Ramp speed update — only call setSpeed when speed changes by >50 µsteps/s
        // to avoid pointless register writes every step during cruise phase.
        if (rampTarget > PITCH_RAMP_START_SPD) {
            const float v0       = PITCH_RAMP_START_SPD;
            const float a        = PITCH_RAMP_ACCEL;
            const float accelSpd = sqrtf(v0*v0 + 2.0f*a*(float)stepsDone);
            const float decelSpd = sqrtf(v0*v0 + 2.0f*a*(float)remaining);
            const float newSpd   = max(v0, min(rampTarget, min(accelSpd, decelSpd)));
            if (fabsf(newSpd - pitch.speed()) > 50.0f) pitch.setSpeed(newSpd);
        }

        pitch.step();
        pitchSteps += fwd ? 1 : -1;
        lastPitchMoveMs = millis();
        remaining--;
        stepsDone++;
    }
    return true;
}

// Move roll by |delta| steps; checks abort signals every step.
static bool moveRollDelta(int32_t delta) {
    if (!USE_ROLL) return true;  // roll bypassed — silently succeed
    if (delta == 0) return true;

    // Wake driver if idle-disabled
    if (AUTO_DISABLE_IDLE && rollIdleOff) {
        roll.enable();
        rollIdleOff = false;
        delay(10);
    }
    const bool cw = delta > 0;
    const uint32_t n = absI32(delta);
    roll.setDirection(cw ? ROLL_CW_DIR : opposite(ROLL_CW_DIR));
    for (uint32_t i = 0; i < n; i++) {
        if (shouldAbortMove()) {
            Serial.println("[move] ABORT roll: safety signal");
            return false;
        }
        roll.step();
        rollSteps += cw ? 1 : -1;
        lastRollMoveMs = millis();
    }
    return true;
}

static const char* modeName(Mode m) {
    switch (m) {
        case Mode::CAN:      return "CAN";
        case Mode::OVERRIDE: return "OVERRIDE";
        case Mode::HOMING:   return "HOMING";
        case Mode::NEUTRAL:  return "NEUTRAL";
    }
    return "?";
}

static uint8_t mapSystemState() {
    if (lastSafety.hard_fault)      return can::STATE_FAULT;
    if (mode == Mode::HOMING)       return can::STATE_HOMING;
    if (!pitchHomed || !rollHomed)  return can::STATE_UNHOMED;
    return can::STATE_HOLD;
}

static bool parseFloatStrict(const String& text, float& out) {
    String t = text;
    t.trim();
    if (t.length() == 0) return false;
    char* end = nullptr;
    out = strtof(t.c_str(), &end);
    while (*end == ' ' || *end == '\t') end++;
    return (*end == '\0') && isfinite(out);
}

// ================================================================
// BMS helpers (encoding for CAN frames)
// ================================================================

// Encode cell mV to (mV-2500)/10 uint8 (matching STATUS_BMS encoding)
static uint8_t encodeCellV(uint16_t mv) {
    if (mv < 2500u) return 0;
    const uint32_t raw = (mv - 2500u) / 10u;
    return (raw > 254u) ? 254u : static_cast<uint8_t>(raw);
}

// Map TinyBMS online_status word to compact 0-5 enum for STATUS_BMS_MORE
static uint8_t mapBmsStatus(uint16_t s) {
    switch (s) {
        case 0x0091: return 1;  // Charging
        case 0x0092: return 2;  // FullyCharged
        case 0x0093: return 3;  // Discharging
        case 0x0096: return 3;  // Regeneration → Discharging
        case 0x0097: return 4;  // Idle
        case 0x009B: return 5;  // Fault
        default:     return 0;  // Unknown
    }
}

// ================================================================
// Homing sequences
// ================================================================

// Pitch: two-end limit-switch homing, centre and zero.
// Includes driver reset between phases (prevents OTP cascade fault).
// Post-centering sanity check detects open-loop stall.

// Called inside blocking homing loops to keep CAN status fresh.
// Polls ToF at its normal cadence and broadcasts STATUS_CONTROL so the Pi
// can track position and ToF distance while homing is in progress.
// Called in the P1/P2 step loops every 16 steps — keeps CAN status fresh without
// blocking I2C reads that would stutter the step timing. ToF value sent is the
// last reading from before homing started; position (pitchSteps) is live.
// CAN broadcast rate during blocking moves/homing — much lower than normal 50Hz
// to avoid frequent SPI transfers that stutter step timing.
static constexpr uint32_t TICK_MOVE_CAN_MS = 200;  // 5Hz during blocking moves

static void tickHoming() {
    const uint32_t t = millis();
    static uint32_t lastHomingTx = 0;
    if (t - lastHomingTx >= TICK_MOVE_CAN_MS) {
        lastHomingTx = t;
        sendStatusControl();
    }
}

static bool homePitch() {
    Serial.println();
    Serial.println(">>> PITCH HOMING START");
    pitchHomed        = false;
    pitchHomingFailed = false;
    bool ok = false;

    wakeDrivers();  // ensure driver is on before direct pitch.step() calls in P1/P2
    pitch.setSpeed(PITCH_HOME_SPEED);

    do {
        // ----------------------------------------------------------
        // P0: choose which limit to seek first.
        // Default is FWD first. If ToF direction is known and the current
        // reading puts us closer to the BWD end, seek BWD first instead —
        // saves up to half the travel on a re-home.
        // ----------------------------------------------------------
        // Sensor faces the forward plate: low ToF = near FWD end, high ToF = near BWD end.
        // If ToF is valid, seek the nearer limit first to save travel time.
        const bool seekFwdFirst = !USE_TOF || !tofValid || (tofMm < TOF_NEUTRAL_MM);
        Serial.print("[P0] ToF="); Serial.print(tofValid ? tofMm : 0);
        Serial.println(seekFwdFirst ? "mm → seeking FWD first"
                                    : "mm → seeking BWD first (ToF shortcut)");

        // ----------------------------------------------------------
        // P1: move toward the FIRST limit (FWD or BWD per P0 decision)
        // ----------------------------------------------------------
        const bool p1Fwd = seekFwdFirst;
        pitch.setDirection(p1Fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
        Serial.println(p1Fwd ? "[P1] seeking FWD limit..." : "[P1] seeking BWD limit...");

        uint32_t moved = 0;
        pitch.setSpeed(PITCH_RAMP_START_SPD);  // start ramp from bottom
        while (!(p1Fwd ? pitchFwdLimit.isPressed() : pitchRevLimit.isPressed())
               && moved < PITCH_HOME_MAX_STEPS) {
            if (USE_PITCH_FLT && pitch.faultActive()) { Serial.println("[P1] ABORT: driver fault"); break; }
            if (leak.isLeak() || inj_leak)            { Serial.println("[P1] ABORT: leak");         break; }
            // Acceleration ramp only — no decel since limit switch position is unknown
            if (PITCH_HOME_SPEED > PITCH_RAMP_START_SPD) {
                const float spd = sqrtf(PITCH_RAMP_START_SPD*PITCH_RAMP_START_SPD + 2.0f*PITCH_RAMP_ACCEL*(float)moved);
                pitch.setSpeed(min(PITCH_HOME_SPEED, spd));
            }
            pitch.step();
            pitchSteps += p1Fwd ? 1 : -1;
            moved++;
            if ((moved & 15) == 0) tickHoming();
        }
        if (!(p1Fwd ? pitchFwdLimit.isPressed() : pitchRevLimit.isPressed())) {
            Serial.print("[P1] FAILED after "); Serial.print(moved);
            Serial.println(p1Fwd ? " steps — FWD limit not reached" : " steps — BWD limit not reached");
            break;
        }
        const int32_t firstHit = pitchSteps;
        Serial.print(p1Fwd ? "[P1] FWD limit at step " : "[P1] BWD limit at step ");
        Serial.println(firstHit);

        // Reset driver between phases — clears any thermal latch from P1
        Serial.println("[P1] resetting driver before reversal...");
        resetPitchDriver();
        if (USE_PITCH_FLT && pitch.faultActive()) { Serial.println("[P1] ABORT: fault persists after reset"); break; }

        // ----------------------------------------------------------
        // P2: move toward the SECOND limit (opposite of P1)
        // ----------------------------------------------------------
        const bool p2Fwd = !seekFwdFirst;
        pitch.setDirection(p2Fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
        Serial.println(p2Fwd ? "[P2] seeking FWD limit..." : "[P2] seeking BWD limit...");

        moved = 0;
        pitch.setSpeed(PITCH_RAMP_START_SPD);  // restart ramp from bottom after driver reset
        while (!(p2Fwd ? pitchFwdLimit.isPressed() : pitchRevLimit.isPressed())
               && moved < PITCH_HOME_MAX_STEPS) {
            if (USE_PITCH_FLT && pitch.faultActive()) { Serial.println("[P2] ABORT: driver fault"); break; }
            if (leak.isLeak() || inj_leak)            { Serial.println("[P2] ABORT: leak");         break; }
            // Acceleration ramp only — no decel since limit switch position is unknown
            if (PITCH_HOME_SPEED > PITCH_RAMP_START_SPD) {
                const float spd = sqrtf(PITCH_RAMP_START_SPD*PITCH_RAMP_START_SPD + 2.0f*PITCH_RAMP_ACCEL*(float)moved);
                pitch.setSpeed(min(PITCH_HOME_SPEED, spd));
            }
            pitch.step();
            pitchSteps += p2Fwd ? 1 : -1;
            moved++;
            if ((moved & 15) == 0) tickHoming();
        }
        if (!(p2Fwd ? pitchFwdLimit.isPressed() : pitchRevLimit.isPressed())) {
            Serial.print("[P2] FAILED after "); Serial.print(moved);
            Serial.println(p2Fwd ? " steps — FWD limit not reached" : " steps — BWD limit not reached");
            break;
        }
        const int32_t secondHit = pitchSteps;
        Serial.print(p2Fwd ? "[P2] FWD limit at step " : "[P2] BWD limit at step ");
        Serial.println(secondHit);

        // Map back to named fwdHit / revHit regardless of search order
        const int32_t fwdHit = seekFwdFirst ? firstHit : secondHit;
        const int32_t revHit = seekFwdFirst ? secondHit : firstHit;

        const int32_t span = fwdHit - revHit;
        Serial.print("[P2] Span = "); Serial.print(span);
        Serial.print(" steps = "); Serial.print(span * config::PITCH_STEP_MM, 2); Serial.println(" mm");
        if (span <= 0) { Serial.println("[P2] FAILED: invalid span"); break; }

        // Reset driver again before centering move
        Serial.println("[P2] resetting driver before centering...");
        resetPitchDriver();
        if (USE_PITCH_FLT && pitch.faultActive()) { Serial.println("[P2] ABORT: fault persists after reset"); break; }

        // ----------------------------------------------------------
        // P3: move to centre, zero the counter
        // ----------------------------------------------------------
        const int32_t center = revHit + (span / 2);
        const int32_t delta  = center - pitchSteps;
        Serial.print("[P3] moving to centre step "); Serial.println(center);

        if (!movePitchDelta(delta)) {
            Serial.println("[P3] ABORT: could not reach centre");
            break;
        }

        pitchSteps = 0;
        pitch.setDirection(PITCH_FORWARD_DIR);

        // Report physical limit switch positions relative to centre — use this to tune PITCH_MAX_MM.
        {
            const float halfSpanMm = (span / 2) * config::PITCH_STEP_MM;
            Serial.println("--------------------------------------------");
            Serial.print("[P3] Physical FWD limit: +"); Serial.print(halfSpanMm, 2); Serial.println(" mm from centre");
            Serial.print("[P3] Physical BWD limit: -"); Serial.print(halfSpanMm, 2); Serial.println(" mm from centre");
            Serial.print("[P3] Software PITCH_MAX_MM = ±"); Serial.print(config::PITCH_MAX_MM, 1); Serial.println(" mm");
            Serial.print("[P3] Using "); Serial.print((config::PITCH_MAX_MM / halfSpanMm) * 100.0f, 1);
            Serial.println("% of physical range — increase PITCH_MAX_MM in constants.hpp to use more");
            Serial.println("--------------------------------------------");
        }

        // Sanity check: at claimed centre neither limit should be pressed.
        // If BWD is still pressed, the motor never left the end-stop — it
        // stalled silently during the centering move. This is the key guard
        // against open-loop position corruption on a loaded leadscrew.
        if (pitchFwdLimit.isPressed()) {
            Serial.println("[P3] SANITY FAIL: FWD limit still pressed at centre — stall during centering. REHOME.");
            break;
        } else if (pitchRevLimit.isPressed()) {
            Serial.println("[P3] SANITY FAIL: BWD limit still pressed at centre — motor did not move. REHOME.");
            break;
        }

        // ----------------------------------------------------------
        // P4: ToF closed-loop correction to neutral (only when USE_TOF)
        // Flush the EMA with live readings first, then jog in 1mm
        // increments until tofMm == TOF_NEUTRAL_MM ± tolerance.
        // Auto-detects which jog direction moves toward target.
        // ----------------------------------------------------------
        if (USE_TOF) {
            Serial.println("[P4] ToF correction to neutral...");

            // Flush EMA — 10 readings × 50ms = 500ms settle
            for (uint8_t i = 0; i < 10; i++) {
                delay(TOF_POLL_MS);
                const ToFSensor::Reading r = tof.read();
                if (r.valid) {
                    tofEma = (i == 0) ? r.mm  // hard-seed first reading
                           : TOF_EMA_ALPHA * r.mm + (1.0f - TOF_EMA_ALPHA) * tofEma;
                    tofMm  = static_cast<uint16_t>(roundf(tofEma));
                    tofValid = true;
                }
            }
            Serial.print("[P4] ToF settled: "); Serial.print(tofMm);
            Serial.print("mm  target: "); Serial.print(TOF_NEUTRAL_MM); Serial.println("mm");

            constexpr int16_t TOF_CORRECT_TOL = 3;  // ±3mm acceptable

            // Sensor faces forward plate: forward move DECREASES ToF.
            // err > 0 → ToF too high → too far back → move FORWARD (step = +1mm)
            // err < 0 → ToF too low  → too far fwd  → move BACKWARD (step = −1mm)
            for (uint8_t attempt = 0; attempt < 30; attempt++) {
                const int16_t err = (int16_t)tofMm - (int16_t)TOF_NEUTRAL_MM;
                if (abs(err) <= TOF_CORRECT_TOL) {
                    Serial.print("[P4] converged: "); Serial.print(tofMm); Serial.println("mm ✓");
                    break;
                }
                const float step = (err > 0) ? 1.0f : -1.0f;
                if (!movePitchDelta(pitchMmToSteps(step))) break;
                delay(100);
                for (uint8_t i = 0; i < 4; i++) {
                    delay(TOF_POLL_MS);
                    const ToFSensor::Reading r = tof.read();
                    if (r.valid) {
                        tofEma = TOF_EMA_ALPHA * r.mm + (1.0f - TOF_EMA_ALPHA) * tofEma;
                        tofMm  = static_cast<uint16_t>(roundf(tofEma));
                    }
                }
            }

            // Zero step counter here — ToF position is the true reference
            pitchSteps = 0;
            Serial.print("[P4] done: ToF="); Serial.print(tofMm);
            Serial.print("mm  err=±"); Serial.print(abs((int16_t)tofMm - (int16_t)TOF_NEUTRAL_MM));
            Serial.println("mm");
        }

        pitchHomed = true;
        ok = true;

    } while (false);

    pitch.setSpeed(PITCH_RUN_SPEED);

    if (ok) {
        Serial.println(">>> PITCH HOMING OK — zeroed at centre");
    } else {
        pitchHomingFailed = true;
        pitchHomed = false;  // ensure bad state never silently persists
        Serial.println(">>> PITCH HOMING FAILED — position unknown, rehome required");
    }
    return ok;
}

// Roll: dual-magnet hall sensor homing, centres between ±60° magnets.
static bool homeRoll() {
    if (!USE_ROLL) {
        // Roll axis not physically wired — fake success so Pi sees roll as homed.
        rollHomed        = true;
        rollHomingFailed = false;
        rollSteps        = 0;
        Serial.println(">>> ROLL HOMING BYPASSED (USE_ROLL=false) — reported as homed");
        return true;
    }
    Serial.println();
    Serial.println(">>> ROLL HOMING START");
    rollHomed        = false;
    rollHomingFailed = false;
    bool ok = false;

    wakeDrivers();  // ensure driver is on before direct roll.step() calls
    roll.setSpeed(ROLL_HOME_SPEED);

    do {
        uint32_t moved = 0;

        // ----------------------------------------------------------
        // R1: CCW until hall active — finds the -30 deg magnet
        // ----------------------------------------------------------
        Serial.println("[R1] CCW seeking -30 deg magnet...");
        roll.setDirection(opposite(ROLL_CW_DIR));

        while (!rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (shouldAbortMove()) { Serial.println("[R1] ABORT: safety signal"); break; }
            roll.step();
            rollSteps--;
            moved++;
        }
        if (!rollHall.isActive()) { Serial.println("[R1] FAILED: -30 magnet not found"); break; }
        const int32_t limitCCW = rollSteps;
        Serial.print("[R1] -30 magnet at step "); Serial.print(limitCCW);
        Serial.print(" ("); Serial.print(limitCCW * config::ROLL_STEP_DEG, 1); Serial.println(" deg)");

        // ----------------------------------------------------------
        // R2: CW — exit -30 magnet, sweep to +30 magnet
        // ----------------------------------------------------------
        Serial.println("[R2] CW — exiting -30, seeking +30...");
        roll.setDirection(ROLL_CW_DIR);
        moved = 0;

        // Exit the -30 magnet first
        while (rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (shouldAbortMove()) { Serial.println("[R2] ABORT: safety signal"); break; }
            roll.step();
            rollSteps++;
            moved++;
        }
        if (rollHall.isActive()) { Serial.println("[R2] FAILED: could not exit -30 magnet"); break; }
        Serial.println("[R2] Exited -30, continuing CW...");

        // Sweep to +30 magnet
        while (!rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (shouldAbortMove()) { Serial.println("[R2] ABORT: safety signal"); break; }
            roll.step();
            rollSteps++;
            moved++;
        }
        if (!rollHall.isActive()) { Serial.println("[R2] FAILED: +30 magnet not found"); break; }
        const int32_t limitCW = rollSteps;
        Serial.print("[R2] +30 magnet at step "); Serial.print(limitCW);
        Serial.print(" ("); Serial.print(limitCW * config::ROLL_STEP_DEG, 1); Serial.println(" deg)");

        const int32_t span = limitCW - limitCCW;
        Serial.print("[R2] Span = "); Serial.print(span * config::ROLL_STEP_DEG, 1); Serial.println(" deg (expect ~60)");

        // ----------------------------------------------------------
        // R3: move to midpoint between the two magnets and zero
        // ----------------------------------------------------------
        const int32_t center = (limitCCW + limitCW) / 2;
        Serial.print("[R3] moving to midpoint step "); Serial.println(center);

        if (!moveRollDelta(center - rollSteps)) {
            Serial.println("[R3] ABORT: could not reach midpoint");
            break;
        }

        rollSteps = 0;
        roll.setDirection(ROLL_CW_DIR);
        rollHomed = true;
        ok = true;

    } while (false);

    roll.setSpeed(ROLL_RUN_SPEED);

    if (ok) {
        Serial.println(">>> ROLL HOMING OK — zeroed at midpoint");
    } else {
        rollHomingFailed = true;
        Serial.println(">>> ROLL HOMING FAILED");
    }
    return ok;
}

// Run both sequences sequentially; called from loop dispatch while mode == HOMING.
static void runHoming() {
    const bool p = homePitch();
    const bool r = homeRoll();
    Serial.print("Homing complete: pitch="); Serial.print(p ? "OK" : "FAIL");
    Serial.print("  roll="); Serial.println(r ? "OK" : "FAIL");
    if (p && r) { targetPitchMm = 0.0f; targetRollDeg = 0.0f; }
    mode = homingReturnMode;

    // Reset CAN timeout — don't count the homing duration against the Pi.
    // The Pi may have gone quiet while waiting for homing to complete;
    // it gets a fresh CMD_TIMEOUT window from this point.
    lastCanRx = millis();

    // Immediately cut holding current on both drivers — the loop is blocked
    // during homing so the idle-disable timer never runs. Without this the
    // motors whine until the next main-loop iteration fires the 1s timer.
    if (AUTO_DISABLE_IDLE) {
        pitch.disable();  pitchIdleOff = true;
        if (USE_ROLL) { roll.disable(); rollIdleOff = true; }
    }
}

// ================================================================
// Setpoint execution
// ================================================================

// ToF closed-loop correction after a pitch setpoint move.
// At home (pitch=0) ToF == TOF_NEUTRAL_MM. Moving forward 1mm closes the gap
// by 1mm, so: targetToF = TOF_NEUTRAL_MM - pitchMm.
// Flushes EMA, then jogs in 1mm steps until within TOF_SETPOINT_TOL.
// Only runs when homed (zero reference is required for the formula to hold).
static void correctPitchWithToF(float targetPitchMm) {
    if (!USE_TOF || !tofValid || !pitchHomed) return;

    const int16_t targetToF = (int16_t)TOF_NEUTRAL_MM - (int16_t)roundf(targetPitchMm);

    // Flush EMA — 5 readings × 50ms = 250ms settle
    for (uint8_t i = 0; i < 5; i++) {
        delay(TOF_POLL_MS);
        const ToFSensor::Reading r = tof.read();
        if (r.valid) {
            tofEma = TOF_EMA_ALPHA * r.mm + (1.0f - TOF_EMA_ALPHA) * tofEma;
            tofMm  = static_cast<uint16_t>(roundf(tofEma));
        }
    }

    constexpr int16_t TOF_SETPOINT_TOL = 2;  // ±2mm
    constexpr uint8_t MAX_CORRECT      = 10; // max 1mm steps before giving up

    for (uint8_t attempt = 0; attempt < MAX_CORRECT; attempt++) {
        const int16_t err = (int16_t)tofMm - targetToF;
        if (abs(err) <= TOF_SETPOINT_TOL) break;
        // err > 0: ToF too high = too far back = move forward
        // err < 0: ToF too low  = too far fwd  = move backward
        const float step = (err > 0) ? 1.0f : -1.0f;
        if (!movePitchDelta(pitchMmToSteps(step))) break;
        delay(100);
        for (uint8_t i = 0; i < 4; i++) {
            delay(TOF_POLL_MS);
            const ToFSensor::Reading r = tof.read();
            if (r.valid) {
                tofEma = TOF_EMA_ALPHA * r.mm + (1.0f - TOF_EMA_ALPHA) * tofEma;
                tofMm  = static_cast<uint16_t>(roundf(tofEma));
            }
        }
    }

    Serial.print("[tof-corr] tgt="); Serial.print(targetPitchMm, 1);
    Serial.print("mm  tofTgt="); Serial.print(targetToF);
    Serial.print("mm  tofNow="); Serial.print(tofMm);
    Serial.print("mm  err=");    Serial.println((int16_t)tofMm - targetToF);
}

// Move both axes to the requested absolute position.
// Clamps to configured limits; checks safety every step via primitives.
// Homing is not required — step 0 is treated as the reference (boot position).
// When homed and ToF is valid, pitch position is closed-loop corrected after
// the step-based move to account for any skipped steps or load-induced error.
static void executeSetpoint(float pitchMm, float rollDeg) {

    const float pClamped = constrain(pitchMm, config::PITCH_MIN_MM, config::PITCH_MAX_MM);
    const float rClamped = constrain(rollDeg, config::ROLL_MIN_DEG, config::ROLL_MAX_DEG);

    const int32_t pDelta = pitchMmToSteps(pClamped) - pitchSteps;
    const int32_t rDelta = rollDegToSteps(rClamped)  - rollSteps;

    bool pitchCompleted = true;
    if (pDelta != 0) { pitch.setSpeed(PITCH_RUN_SPEED); pitchCompleted = movePitchDelta(pDelta); }
    if (rDelta != 0) { roll.setSpeed(ROLL_RUN_SPEED);   moveRollDelta(rDelta);  }

    // Closed-loop ToF correction — only when pitch move completed fully.
    // If a limit switch or fault stopped the move early, skip correction:
    // the motor is at a physical boundary and can't reach the target anyway.
    // Skip ToF correction if a new setpoint arrived mid-move — no point correcting
    // to an old target we're about to leave anyway.
    if (pDelta != 0 && pitchCompleted && !newSetpointPending) correctPitchWithToF(pClamped);
}

// ================================================================
// Jog helpers
// ================================================================

// Jog pitch by ±mm from current position. Uses pitchJogSpeed (runtime settable).
// Single movePitchDelta call — the trapezoidal ramp handles smooth accel and
// decel, ramping back down to PITCH_RAMP_START_SPD in the last ~1-2mm.
// Real stall/OCP protection is inside movePitchDelta (fault pin + auto-retry).
// ToF delta printed after move so you can verify actual vs commanded travel.

// Jog roll by ±deg from current position. Uses rollJogSpeed (runtime settable).
static void jogRoll(float deg) {
    if (deg == 0.0f) return;
    const int32_t steps = rollDegToSteps(deg);
    if (steps == 0) return;
    roll.setSpeed(rollJogSpeed);
    moveRollDelta(steps);
    roll.setSpeed(ROLL_RUN_SPEED);  // restore run speed afterwards

    Serial.print("[jog] roll ");
    Serial.print(deg > 0 ? "+" : "");
    Serial.print(deg, 2);
    Serial.print(" deg  pos=");
    Serial.print(rollStepsToDeg(rollSteps), 3);
    Serial.println(" deg");
}

// ================================================================
// CAN TX
// ================================================================

static bool isFrameDifferentIgnoreSeq(const CanFrame& a, const CanFrame& b, uint8_t seqIdx) {
    if (a.id != b.id || a.len != b.len) return true;
    for (uint8_t i = 0; i < a.len; i++) {
        if (i == seqIdx) continue;
        if (a.data[i] != b.data[i]) return true;
    }
    return false;
}

// STATUS_CONTROL (0x203 + node_id) — 50 Hz
// pitch_pos_01mm: ToF-derived when homed+valid (slip-immune), falls back to step count.
// Formula: pitchMm = TOF_NEUTRAL_MM - tofEma  (forward move closes gap → ToF decreases)
static void sendStatusControl() {
    can::StatusControl msg{};
    const float pitchMm_report = (USE_TOF && tofValid && pitchHomed)
        ? ((float)TOF_NEUTRAL_MM - tofEma)
        : pitchStepsToMm(pitchSteps);
    msg.pitch_pos_01mm   = (int16_t)constrain((int)roundf(pitchMm_report * 10.0f), -32768, 32767);
    msg.roll_angle_01deg = (int16_t)constrain((int)roundf(rollStepsToDeg(rollSteps)  * 10.0f), -32768, 32767);

    uint8_t fa = 0;
    if (pitchHomed)                           fa |= can::FLAG_HOMED_P;
    if (rollHomed)                            fa |= can::FLAG_HOMED_R;
    if (motorsEnabled && !pitchIdleOff)       fa |= can::FLAG_ENABLE_P;
    if (USE_ROLL && motorsEnabled && !rollIdleOff) fa |= can::FLAG_ENABLE_R;
    msg.flagsA = fa;

    uint8_t fb = 0;
    if (pitchFwdLimit.isPressed())  fb |= can::FLAG_LIM_F;
    if (pitchRevLimit.isPressed())  fb |= can::FLAG_LIM_R;
    if (leak.isLeak() || inj_leak) fb |= can::FLAG_LEAK;
    if (rollHall.isActive())        fb |= can::FLAG_HALL;
    msg.flagsB = fb;

    msg.sequence = canSeqControl++;
    msg.tof_mm   = (USE_TOF && tofValid) ? tofMm : 0;

    uint8_t buf[8];
    can::packStatusControl(buf, msg);
    CanFrame frame;
    frame.id  = can::STATUS_CONTROL_BASE + CAN_NODE_ID;
    frame.len = 8;
    for (int i = 0; i < 8; i++) frame.data[i] = buf[i];
    canBus.write(frame);
}

// STATUS_FAULT (0x053 + node_id) — 2 Hz
static void sendStatusFault() {
    can::StatusFault msg{};
    msg.hard_fault_a     = lastSafety.hard_fault_a;
    msg.hard_fault_b     = lastSafety.hard_fault_b;
    msg.soft_fault       = lastSafety.soft_fault_bits;
    msg.first_hard_fault = lastSafety.first_hard_fault;
    msg.state            = mapSystemState();
    msg.sequence         = canSeqFault++;

    uint8_t buf[8];
    can::packStatusFault(buf, msg);
    CanFrame frame;
    frame.id  = can::STATUS_FAULT_BASE + CAN_NODE_ID;
    frame.len = 8;
    for (int i = 0; i < 8; i++) frame.data[i] = buf[i];
    canBus.write(frame);
}

// STATUS_BMS (0x213 + node_id) — 2 Hz when USE_BMS
static void sendStatusBms() {
    const BmsManager::Telemetry& t = bmsManager.telemetry();
    can::StatusBMS msg{};

    // Pack voltage: float V → uint8 ×0.1V (0–25.5 V)
    { float v = t.pack_voltage_v * 10.0f; msg.pack_V = (uint8_t)constrain((int)v, 0, 255); }

    msg.min_cell_V = encodeCellV(t.min_cell_mv);
    msg.max_cell_V = encodeCellV(t.max_cell_mv);

    // Pack current: float A → int8 ×0.5A (+ = discharge)
    { float c = t.pack_current_a / 0.5f; msg.pack_current = (int8_t)constrain((int)c, -128, 127); }

    // Temperature: 0.1°C int16 → uint8 °C; 0xFF = not connected
    auto encodeTemp = [](int16_t dC) -> uint8_t {
        if (dC == -32768) return 0xFF;
        int32_t degC = dC / 10;
        if (degC < 0) return 0;
        if (degC > 254) return 254;
        return static_cast<uint8_t>(degC);
    };
    msg.temp_internal = encodeTemp(t.pack_temp_dC);
    msg.temp_ext1     = encodeTemp(t.ext1_temp_dC);
    msg.temp_ext2     = encodeTemp(t.ext2_temp_dC);
    msg.sequence      = canSeqBms++;

    uint8_t buf[8];
    can::packStatusBMS(buf, msg);
    CanFrame frame;
    frame.id  = can::STATUS_BMS_BASE + CAN_NODE_ID;
    frame.len = 8;
    for (int i = 0; i < 8; i++) frame.data[i] = buf[i];
    canBus.write(frame);
}

// STATUS_BMS_MORE (0x223 + node_id) — 2 Hz when USE_BMS, first 6 individual cell voltages
static void sendStatusBmsMore() {
    const BmsManager::Telemetry& t = bmsManager.telemetry();
    can::StatusBmsMore msg{};

    const uint8_t cells = (t.num_cells > 6) ? 6 : t.num_cells;
    for (uint8_t i = 0; i < cells; i++) msg.cell_V[i] = encodeCellV(t.cell_mv[i]);

    msg.online_status = mapBmsStatus(t.online_status);
    msg.sequence      = canSeqBmsMore++;

    uint8_t buf[8];
    can::packStatusBmsMore(buf, msg);
    CanFrame frame;
    frame.id  = can::STATUS_BMS_MORE_BASE + CAN_NODE_ID;
    frame.len = 8;
    for (int i = 0; i < 8; i++) frame.data[i] = buf[i];
    canBus.write(frame);
}

// ================================================================
// CAN RX
// ================================================================

static void handleCan() {
    CanFrame frame;
    while (canBus.read(frame)) {
        const uint16_t id = frame.id & 0x7FF;

        // Print any changed frame when debug is on (ignores sequence byte at idx 6)
        if (debugPrint) {
            constexpr uint8_t SEQ_IDX = 6;
            if (!lastFrames[id].valid ||
                isFrameDifferentIgnoreSeq(frame, lastFrames[id].frame, SEQ_IDX)) {
                Serial.print("RX id=0x"); Serial.print(frame.id, HEX);
                Serial.print(" len=");    Serial.print(frame.len);
                Serial.print(" data=");
                for (uint8_t i = 0; i < frame.len; i++) { Serial.print(frame.data[i]); Serial.print(" "); }
                Serial.println();
                lastFrames[id].frame = frame;
                lastFrames[id].valid = true;
            }
        }

        if (id != can::CMD_SETPOINT_BASE + CAN_NODE_ID) continue;

        lastCanRx = millis();
        canRxEver = true;

        can::CmdSetpoint cmd;
        can::unpackCmdSetpoint(frame.data, cmd);

        // Homing command — trigger on rising edge only (0→1 transition).
        // Pi may hold CMD_START_HOMING high for multiple frames; without edge
        // detection homing would re-trigger immediately after completing.
        static bool lastHomingBit = false;
        const bool homingBit = (cmd.command_mode & can::CMD_START_HOMING) != 0;
        const bool homingRising = homingBit && !lastHomingBit;
        lastHomingBit = homingBit;

        if (homingRising && mode != Mode::HOMING) {
            homingReturnMode = Mode::CAN;
            mode = Mode::HOMING;
            Serial.println("CAN: homing triggered");
            return;
        }

        // Pi reconnected after CAN timeout — resume CAN mode
        if (mode == Mode::NEUTRAL) {
            mode = Mode::CAN;
            lastCanRx = millis();
            Serial.println("CAN reconnected — resuming CAN mode");
        }

        if (mode != Mode::CAN) continue;

        // Decode setpoint (01mm and 01deg units)
        const float pMm  = cmd.pitch_travel_01mm * 0.1f;
        const float rDeg = cmd.roll_angle_01deg   * 0.1f;

        // Only flag as pending when the value actually changed
        static int16_t lastPitchRaw = INT16_MIN;
        static int16_t lastRollRaw  = INT16_MIN;

        if (cmd.pitch_travel_01mm != lastPitchRaw || cmd.roll_angle_01deg != lastRollRaw) {
            targetPitchMm      = pMm;
            targetRollDeg      = rDeg;
            newSetpointPending = true;
            lastPitchRaw       = cmd.pitch_travel_01mm;
            lastRollRaw        = cmd.roll_angle_01deg;
        }
    }
}

// ================================================================
// Serial CLI
// ================================================================

static void clearInjections() {
    inj_leak = inj_driver_p = inj_driver_r = false;
    inj_cmd_s = inj_cmd_h = false;
    inj_bms_to_s = inj_bms_to_h = false;
    inj_bms_temp = inj_bms_oc = inj_bms_sw = false;
    inj_bms_low = inj_bms_high = false;
    inj_home_fail_p = inj_home_fail_r = false;
}

static void printInjections() {
    Serial.print("INJ leak=");    Serial.print(inj_leak ? 1 : 0);
    Serial.print(" drvP=");       Serial.print(inj_driver_p ? 1 : 0);
    Serial.print(" drvR=");       Serial.print(inj_driver_r ? 1 : 0);
    Serial.print(" scmd=");       Serial.print(inj_cmd_s ? 1 : 0);
    Serial.print(" hcmd=");       Serial.print(inj_cmd_h ? 1 : 0);
    Serial.print(" sbto=");       Serial.print(inj_bms_to_s ? 1 : 0);
    Serial.print(" hbto=");       Serial.print(inj_bms_to_h ? 1 : 0);
    Serial.print(" btemp=");      Serial.print(inj_bms_temp ? 1 : 0);
    Serial.print(" boc=");        Serial.print(inj_bms_oc ? 1 : 0);
    Serial.print(" bsw=");        Serial.print(inj_bms_sw ? 1 : 0);
    Serial.print(" blow=");       Serial.print(inj_bms_low ? 1 : 0);
    Serial.print(" bhigh=");      Serial.print(inj_bms_high ? 1 : 0);
    Serial.print(" hfailP=");     Serial.print(inj_home_fail_p ? 1 : 0);
    Serial.print(" hfailR=");     Serial.println(inj_home_fail_r ? 1 : 0);
}

static void printHelp() {
    Serial.println("\n=== P&R Commands ===");
    Serial.println("-- Always available --");
    Serial.println("auto              -> enter OVERRIDE (serial control)");
    Serial.println("debug t / f       -> enable / disable periodic status print");
    Serial.println("status            -> print status once");
    Serial.println("help              -> this message");
    Serial.println("-- OVERRIDE mode --");
    Serial.println("home              -> home pitch + roll");
    Serial.println("homep             -> home pitch only");
    Serial.println("homer             -> home roll only");
    Serial.println("en  / dis         -> enable / disable motors");
    Serial.println("ap <mm>           -> pitch absolute position (mm, from centre)");
    Serial.println("ar <deg>          -> roll absolute angle (deg, from centre)");
    Serial.println("goto <mm> <deg>   -> pitch + roll absolute in one command");
    Serial.println("f<mm>             -> pitch to absolute +mm from neutral  e.g. f10, f 10");
    Serial.println("b<mm>             -> pitch to absolute -mm from neutral  e.g. b30, b 30");
    Serial.println("rcw <deg>         -> jog roll CW N degrees");
    Serial.println("rccw <deg>        -> jog roll CCW N degrees");
    Serial.println("pspd <steps/s>    -> set pitch jog speed (lower = more torque)");
    Serial.println("rspd <steps/s>    -> set roll jog speed");
    Serial.println("c                 -> return to CAN mode (requires homed)");
    Serial.println("recover           -> clear latched safety faults");
    Serial.println("inj / inj0        -> print / clear fault injections");
    Serial.println("-- Hard fault injections --");
    Serial.println("leak1/0  drvp1/0  drvr1/0  hcmd1/0  hbto1/0");
    Serial.println("btemp1/0  boc1/0  bsw1/0  hfailp1/0  hfailr1/0");
    Serial.println("-- Soft fault injections --");
    Serial.println("scmd1/0  sbto1/0  blow1/0  bhigh1/0");
    Serial.println();
}

static void printStatus() {
    Serial.println();
    Serial.print("[NODE="); Serial.print(CAN_NODE_ID);
    Serial.print(" BMS=");   Serial.print(USE_BMS ? "EN" : "DIS");
    Serial.print("]  mode="); Serial.print(modeName(mode));
    Serial.print("  pitch_drv="); Serial.print(!motorsEnabled ? "DIS" : pitchIdleOff ? "IDLE-OFF" : "EN");
    Serial.print("  roll_drv=");  Serial.print(!USE_ROLL ? "BYPASSED" : !motorsEnabled ? "DIS" : rollIdleOff ? "IDLE-OFF" : "EN");
    Serial.print("  homed_P="); Serial.print(pitchHomed ? "Y" : "N");
    Serial.print("  homed_R="); Serial.println(rollHomed ? "Y" : "N");

    Serial.print("  pitch=");
    if (pitchHomed) { Serial.print(pitchStepsToMm(pitchSteps), 3); Serial.print("mm"); }
    else Serial.print("UNKNOWN");
    Serial.print("  tgt=");     Serial.print(targetPitchMm, 2);
    Serial.print("mm  lim_F="); Serial.print(pitchFwdLimit.isPressed() ? "PRESSED" : "open");
    Serial.print("  lim_R=");   Serial.print(pitchRevLimit.isPressed() ? "PRESSED" : "open");
    Serial.print("  flt_P=");   Serial.println(pitch.faultActive() ? "FAULT" : "ok");

    Serial.print("  roll=");
    if (!USE_ROLL) {
        Serial.println("BYPASSED (USE_ROLL=false)");
    } else {
        if (rollHomed) { Serial.print(rollStepsToDeg(rollSteps), 3); Serial.print("deg"); }
        else Serial.print("UNKNOWN");
        Serial.print("  tgt=");    Serial.print(targetRollDeg, 2);
        Serial.print("deg  hall="); Serial.print(rollHall.isActive() ? "ACTIVE" : "inactive");
        Serial.print("  flt_R=");   Serial.println(roll.faultActive() ? "FAULT" : "ok");
    }

    if (USE_TOF) {
        Serial.print("  tof=");
        if (tofValid) { Serial.print(tofMm); Serial.print("mm (neutral~"); Serial.print(TOF_NEUTRAL_MM); Serial.print("mm)"); }
        else Serial.print("invalid");
        Serial.print("  pspd="); Serial.print(pitchJogSpeed, 0); Serial.println(" steps/s");
    }
    Serial.print("  leak=");    Serial.print(leak.isLeak() ? "LEAK" : "ok");
    Serial.print("  HF_A=0x"); Serial.print(lastSafety.hard_fault_a, HEX);
    Serial.print("  HF_B=0x"); Serial.print(lastSafety.hard_fault_b, HEX);
    Serial.print("  SF=0x");   Serial.print(lastSafety.soft_fault_bits, HEX);
    Serial.print("  first=");  Serial.println(lastSafety.first_hard_fault);

    if (USE_BMS) {
        const BmsManager::Telemetry& tel = bmsManager.telemetry();
        Serial.print("  BMS V=");  Serial.print(tel.pack_voltage_v, 2);
        Serial.print("V  I=");     Serial.print(tel.pack_current_a, 2);
        Serial.print("A  T=");     Serial.print(tel.pack_temp_dC / 10.0f, 1);
        Serial.print("C  snap=");  Serial.println(tel.full_snapshot_ready ? "Y" : "N (loading)");
    }
}

static void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    // ---- Always available ----
    if (cmd == "help")    { printHelp();   return; }
    if (cmd == "status")  { printStatus(); return; }
    if (cmd == "debug t") { debugPrint = true;  Serial.println("Debug ON");  return; }
    if (cmd == "debug f") { debugPrint = false; Serial.println("Debug OFF"); return; }

    if (cmd == "auto") {
        if (mode == Mode::HOMING) { Serial.println("Cannot override during homing"); return; }
        mode = Mode::OVERRIDE;
        Serial.println("OVERRIDE mode — type 'help' for commands");
        return;
    }

    // ---- OVERRIDE only ----
    if (mode != Mode::OVERRIDE) {
        Serial.println("Not in OVERRIDE — send 'auto' first");
        return;
    }

    if (cmd == "en") {
        if (lastSafety.hard_fault) { Serial.println("Hard fault active — run 'recover' first"); return; }
        pitch.enable();
        if (USE_ROLL) roll.enable();
        motorsEnabled = true;
        pitchIdleOff = false;
        rollIdleOff  = false;
        lastPitchMoveMs = millis();
        lastRollMoveMs  = millis();
        Serial.println("Motors enabled");
        return;
    }
    if (cmd == "dis") {
        pitch.disable();
        if (USE_ROLL) roll.disable();
        motorsEnabled = false;
        pitchIdleOff = false;
        rollIdleOff  = false;
        Serial.println("Motors disabled");
        return;
    }

    if (cmd == "homep") { homePitch(); printStatus(); return; }
    if (cmd == "homer") { homeRoll();  printStatus(); return; }
    if (cmd == "home") {
        const bool p = homePitch();
        const bool r = homeRoll();
        Serial.print("Home: pitch="); Serial.print(p ? "OK" : "FAIL");
        Serial.print("  roll="); Serial.println(r ? "OK" : "FAIL");
        if (p && r) { targetPitchMm = 0.0f; targetRollDeg = 0.0f; }
        printStatus();
        return;
    }

    // Absolute pitch move
    if (cmd.startsWith("ap")) {
        if (!pitchHomed) { Serial.println("Pitch not homed"); return; }
        float mm;
        if (!parseFloatStrict(cmd.substring(2), mm)) { Serial.println("Usage: ap <mm>"); return; }
        executeSetpoint(mm, rollStepsToDeg(rollSteps));
        printStatus();
        return;
    }

    // Absolute roll move
    if (cmd.startsWith("ar")) {
        if (!rollHomed) { Serial.println("Roll not homed"); return; }
        float deg;
        if (!parseFloatStrict(cmd.substring(2), deg)) { Serial.println("Usage: ar <deg>"); return; }
        executeSetpoint(pitchStepsToMm(pitchSteps), deg);
        printStatus();
        return;
    }

    // Absolute pitch + roll in one command
    if (cmd.startsWith("goto")) {
        if (!pitchHomed || !rollHomed) { Serial.println("Not homed"); return; }
        String args = cmd.substring(4);
        args.trim();
        const int sep = args.indexOf(' ');
        if (sep < 0) { Serial.println("Usage: goto <mm> <deg>"); return; }
        float mm, deg;
        if (!parseFloatStrict(args.substring(0, sep), mm) ||
            !parseFloatStrict(args.substring(sep + 1), deg)) {
            Serial.println("Bad values");
            return;
        }
        executeSetpoint(mm, deg);
        printStatus();
        return;
    }

    // Absolute pitch position commands — f<mm> = forward from neutral, b<mm> = backward.
    // e.g. "f10" or "f 10" → +10mm, "b30" or "b 30" → −30mm, "f0" → neutral.
    if (cmd.startsWith("f") && cmd.length() > 1) {
        if (!pitchHomed) { Serial.println("Not homed — run 'home' first"); return; }
        const float target = cmd.substring(1).toFloat();
        if (target < 0.0f || target > config::PITCH_MAX_MM) {
            Serial.print("Out of range: f0 to f"); Serial.println(config::PITCH_MAX_MM, 1);
            return;
        }
        const float currentMm = (USE_TOF && tofValid && pitchHomed)
            ? ((float)TOF_NEUTRAL_MM - tofEma) : pitchStepsToMm(pitchSteps);
        const float deltaMm = target - currentMm;
        Serial.print("[pitch] abs target=+"); Serial.print(target, 2);
        Serial.print("mm  current="); Serial.print(currentMm, 2); Serial.print("mm");
        pitch.setSpeed(pitchJogSpeed);
        const bool ok = movePitchDelta(pitchMmToSteps(deltaMm));
        pitch.setSpeed(PITCH_RUN_SPEED);
        Serial.print("  tof="); Serial.print(tofMm); Serial.print("mm");
        Serial.print("  pos="); Serial.print(pitchStepsToMm(pitchSteps), 2); Serial.print("mm");
        if (!ok) Serial.print("  [STOPPED EARLY]");
        Serial.println();
        return;
    }
    if (cmd.startsWith("b") && cmd.length() > 1) {
        if (!pitchHomed) { Serial.println("Not homed — run 'home' first"); return; }
        const float target = cmd.substring(1).toFloat();
        if (target < 0.0f || target > config::PITCH_MAX_MM) {
            Serial.print("Out of range: b0 to b"); Serial.println(config::PITCH_MAX_MM, 1);
            return;
        }
        const float currentMm = (USE_TOF && tofValid && pitchHomed)
            ? ((float)TOF_NEUTRAL_MM - tofEma) : pitchStepsToMm(pitchSteps);
        const float deltaMm = -target - currentMm;
        Serial.print("[pitch] abs target=-"); Serial.print(target, 2);
        Serial.print("mm  current="); Serial.print(currentMm, 2); Serial.print("mm");
        pitch.setSpeed(pitchJogSpeed);
        const bool ok = movePitchDelta(pitchMmToSteps(deltaMm));
        pitch.setSpeed(PITCH_RUN_SPEED);
        Serial.print("  tof="); Serial.print(tofMm); Serial.print("mm");
        Serial.print("  pos="); Serial.print(pitchStepsToMm(pitchSteps), 2); Serial.print("mm");
        if (!ok) Serial.print("  [STOPPED EARLY]");
        Serial.println();
        return;
    }
    if (cmd.startsWith("rcw ")) {
        if (!USE_ROLL) { Serial.println("Roll bypassed (USE_ROLL=false)"); return; }
        float deg = cmd.substring(4).toFloat();
        if (deg <= 0.0f) { Serial.println("Usage: rcw <deg>  e.g. rcw 30"); return; }
        jogRoll(deg);
        return;
    }
    if (cmd.startsWith("rccw ")) {
        if (!USE_ROLL) { Serial.println("Roll bypassed (USE_ROLL=false)"); return; }
        float deg = cmd.substring(5).toFloat();
        if (deg <= 0.0f) { Serial.println("Usage: rccw <deg>  e.g. rccw 15"); return; }
        jogRoll(-deg);
        return;
    }

    // Runtime jog speed adjustment (lower speed = more torque for stuck pitch)
    if (cmd.startsWith("pspd ")) {
        float spd = cmd.substring(5).toFloat();
        if (spd < 800.0f || spd > 24000.0f) { Serial.println("pspd: valid range 800–24000 microsteps/s"); return; }
        pitchJogSpeed = spd;
        Serial.print("Pitch jog speed set to "); Serial.print(spd, 0); Serial.println(" steps/s");
        return;
    }
    if (cmd.startsWith("rspd ")) {
        float spd = cmd.substring(5).toFloat();
        if (spd < 50.0f || spd > 2000.0f) { Serial.println("rspd: valid range 50–2000 steps/s"); return; }
        rollJogSpeed = spd;
        Serial.print("Roll jog speed set to "); Serial.print(spd, 0); Serial.println(" steps/s");
        return;
    }

    if (cmd == "c") {
        mode = Mode::CAN;
        lastCanRx = millis();
        Serial.println("CAN mode");
        return;
    }

    if (cmd == "recover") {
        safety.reset(); lastSafety = {};
        pitchHomingFailed = false; rollHomingFailed = false;
        // Clear DRV8825 hardware fault latch (EN toggle) — drivers are enabled after this
        resetPitchDriver();
        resetRollDriver();
        pitchIdleOff = false;  // resetPitchDriver() left the driver enabled
        rollIdleOff  = false;
        lastPitchMoveMs = millis();
        lastRollMoveMs  = millis();
        motorsEnabled = true;
        Serial.println("Safety + driver faults reset");
        return;
    }

    if (cmd == "inj0") { clearInjections(); Serial.println("Injections cleared"); return; }
    if (cmd == "inj")  { printInjections(); return; }

    // ---- Hard fault injections ----
    if (cmd == "leak1")   { inj_leak = true;          return; }
    if (cmd == "leak0")   { inj_leak = false;         return; }
    if (cmd == "drvp1")   { inj_driver_p = true;      return; }
    if (cmd == "drvp0")   { inj_driver_p = false;     return; }
    if (cmd == "drvr1")   { inj_driver_r = true;      return; }
    if (cmd == "drvr0")   { inj_driver_r = false;     return; }
    if (cmd == "hcmd1")   { inj_cmd_h = true;         return; }
    if (cmd == "hcmd0")   { inj_cmd_h = false;        return; }
    if (cmd == "hbto1")   { inj_bms_to_h = true;      return; }
    if (cmd == "hbto0")   { inj_bms_to_h = false;     return; }
    if (cmd == "btemp1")  { inj_bms_temp = true;      return; }
    if (cmd == "btemp0")  { inj_bms_temp = false;     return; }
    if (cmd == "boc1")    { inj_bms_oc = true;        return; }
    if (cmd == "boc0")    { inj_bms_oc = false;       return; }
    if (cmd == "bsw1")    { inj_bms_sw = true;        return; }
    if (cmd == "bsw0")    { inj_bms_sw = false;       return; }
    if (cmd == "hfailp1") { inj_home_fail_p = true;   return; }
    if (cmd == "hfailp0") { inj_home_fail_p = false;  return; }
    if (cmd == "hfailr1") { inj_home_fail_r = true;   return; }
    if (cmd == "hfailr0") { inj_home_fail_r = false;  return; }

    // ---- Soft fault injections ----
    if (cmd == "scmd1")  { inj_cmd_s = true;       return; }
    if (cmd == "scmd0")  { inj_cmd_s = false;      return; }
    if (cmd == "sbto1")  { inj_bms_to_s = true;    return; }
    if (cmd == "sbto0")  { inj_bms_to_s = false;   return; }
    if (cmd == "blow1")  { inj_bms_low = true;     return; }
    if (cmd == "blow0")  { inj_bms_low = false;    return; }
    if (cmd == "bhigh1") { inj_bms_high = true;    return; }
    if (cmd == "bhigh0") { inj_bms_high = false;   return; }

    // ---- EN pin diagnostic (temporary) ----
    // Bypasses all driver logic and toggles the raw pin directly.
    // 'entest on'  → pin 22 HIGH  (TB67S581FNG: should ENABLE  → motor whines)
    // 'entest off' → pin 22 LOW   (TB67S581FNG: should DISABLE → motor silent)
    if (cmd == "entest on")  {
        pinMode(PIN_ROLL_ENA, OUTPUT);
        digitalWrite(PIN_ROLL_ENA, HIGH);
        Serial.println("EN raw HIGH — motor should be ON (whining) if wired");
        return;
    }
    if (cmd == "entest off") {
        pinMode(PIN_ROLL_ENA, OUTPUT);
        digitalWrite(PIN_ROLL_ENA, LOW);
        Serial.println("EN raw LOW — motor should be OFF (silent) if wired");
        return;
    }

    Serial.print("Unknown: "); Serial.println(cmd);
}

static void readSerial() {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (cmdBuffer.length() > 0) { handleCommand(cmdBuffer); cmdBuffer = ""; }
        } else if (cmdBuffer.length() < 64) {
            cmdBuffer += c;
        } else {
            cmdBuffer = "";
            Serial.println("ERROR: command too long");
        }
    }
}

// ================================================================
// Setup
// ================================================================

void setup() {
    Serial.begin(115200);
    logger::begin(115200);
    cmdBuffer.reserve(64);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Hold pitch driver in sleep+reset during boot.
    // Pi power-up noise can assert FLT before VMOT is stable.
    // RST LOW clears any transient fault latch; SLP LOW disables outputs.
    // Both released once rails are settled.
    pinMode(PIN_PITCH_SLP_RST, OUTPUT);
    digitalWrite(PIN_PITCH_SLP_RST, LOW);   // driver asleep + in reset
    delay(500);                              // wait for Pi / VMOT to stabilise
    digitalWrite(PIN_PITCH_SLP_RST, HIGH);  // wake driver — fault latch cleared by RST
    delay(10);

    pitch.begin();
    if (USE_ROLL) roll.begin();
    pitchFwdLimit.begin();
    pitchRevLimit.begin();
    if (USE_ROLL) rollHall.begin();
    leak.begin(false);

    pitch.setSpeed(PITCH_RUN_SPEED);
    if (USE_ROLL) roll.setSpeed(ROLL_RUN_SPEED);

    // Start with driver disabled — no whine on boot.
    // wakeDrivers() will enable automatically before the first move.
    if (AUTO_DISABLE_IDLE) {
        pitch.disable();
        if (USE_ROLL) roll.disable();
        pitchIdleOff = true;
        rollIdleOff  = USE_ROLL;
    } else {
        pitch.enable();
        if (USE_ROLL) roll.enable();
        pitchIdleOff = false;
        rollIdleOff  = false;
    }
    motorsEnabled = true;
    lastPitchMoveMs = 0;  // ensures idle check fires immediately on boot
    lastRollMoveMs  = 0;

    if (USE_TOF) {
        if (tof.begin(Wire1)) {
            Serial.println("[tof] VL6180X OK");
        } else {
            Serial.println("[tof] VL6180X NOT FOUND — check wiring");
        }
    }

    safety.configure({});
    safety.reset();

    canBus.begin(500000, PIN_CAN_R, PIN_CAN_D);

    if (USE_BMS) {
        bmsUart.begin();
        bmsManager.configure({
            .transport           = &bmsUart,
            .poll_interval_ms    = 100,
            .response_timeout_ms = 120,
            .max_retries         = 1,
            .fault_confirm_ms    = (uint32_t)(config::BMS_TEMP_CONFIRM_S * 1000.0f),
            .timeout_small_ms    = config::BMS_TIMEOUT_SMALL_MS,
            .timeout_large_ms    = config::BMS_TIMEOUT_LARGE_MS
        });
        bmsManager.begin();
    }

    Serial.print("\nP&R ready  [NODE_ID=");
    Serial.print(CAN_NODE_ID);
    Serial.print("  BMS="); Serial.print(USE_BMS ? "EN" : "DIS");
    Serial.println("]");
    Serial.println("CAN mode active. Pi sends CMD_START_HOMING to begin.");
    Serial.println("Type 'auto' for serial control, 'debug t' for live status, 'help' for commands.");
    Serial.println();
}

// ================================================================
// Loop
// ================================================================

void loop() {
    const uint32_t now = millis();

    // LED heartbeat: 100 ms flash every 5 s
    if (!ledOn && now - lastLedToggle >= 5000) {
        digitalWrite(LED_BUILTIN, HIGH); ledOn = true; lastLedToggle = now;
    }
    if (ledOn && now - lastLedToggle >= 100) {
        digitalWrite(LED_BUILTIN, LOW); ledOn = false;
    }

    // dt — capped at 200 ms to avoid windup on a hiccup
    static uint32_t lastTime = 0;
    const float dt = lastTime ? ((now - lastTime) < 200 ? (now - lastTime) * 1e-3f : 0.2f) : 0.0f;
    lastTime = now;

    // BMS (non-blocking UART pump + state machine)
    if (USE_BMS) {
        bmsUart.update();
        bmsManager.update(now);
    }

    // ToF background poll (non-blocking cadence; not called during blocking moves)
    if (USE_TOF && (now - lastTofPoll >= TOF_POLL_MS)) {
        lastTofPoll = now;
        const ToFSensor::Reading r = tof.read();
        if (r.valid) {
            if (!tofValid) {
                tofEma = static_cast<float>(r.mm);  // seed EMA on first valid reading
            } else {
                tofEma = TOF_EMA_ALPHA * static_cast<float>(r.mm)
                       + (1.0f - TOF_EMA_ALPHA) * tofEma;
            }
            tofMm    = static_cast<uint16_t>(roundf(tofEma));
            tofValid = true;

            // Sync step counter from ToF — corrects accumulated slip.
            // pitchMm = TOF_NEUTRAL_MM - tofEma (forward move closes gap).
            // Only apply when homed; during homing pitchSteps has a different meaning.
            if (pitchHomed) {
                pitchSteps = pitchMmToSteps((float)TOF_NEUTRAL_MM - tofEma);
            }
        }
    }

    // Serial + CAN input processing
    readSerial();
    handleCan();

    // Large CAN timeout: move to NEUTRAL (go to pitch=0 roll=0).
    // Only meaningful when homed — without a home reference there is no safe zero to drive to.
    if (mode == Mode::CAN && canRxEver && pitchHomed && rollHomed &&
        (millis() - lastCanRx > config::CMD_TIMEOUT_LARGE_MS)) {
        mode = Mode::NEUTRAL;
        newSetpointPending = false;
        Serial.println("!! CAN timeout — entering NEUTRAL (pitch=0 roll=0)");
    }

    // ----------------------------------------------------------------
    // Mode dispatch  (blocking step-based moves happen here)
    // ----------------------------------------------------------------
    if (mode == Mode::HOMING) {
        runHoming();

    } else if (mode == Mode::NEUTRAL) {
        // Drive to zero if not already there
        if (pitchHomed && rollHomed) {
            const float curP = pitchStepsToMm(pitchSteps);
            const float curR = rollStepsToDeg(rollSteps);
            if (fabsf(curP) > 0.1f || fabsf(curR) > 0.5f) {
                executeSetpoint(0.0f, 0.0f);
                Serial.println("NEUTRAL: reached pitch=0 roll=0");
            }
        }

    } else if (newSetpointPending && mode == Mode::CAN) {
        // Execute a setpoint received from the Pi — homing not required
        if (!lastSafety.hard_fault) {
            newSetpointPending = false;
            executeSetpoint(targetPitchMm, targetRollDeg);
        }
    }

    // ----------------------------------------------------------------
    // Safety manager update
    // Re-read millis() — blocking moves above may have taken many ms.
    // ----------------------------------------------------------------
    {
        const uint32_t now2    = millis();
        const bool inCanFam    = (mode == Mode::CAN || mode == Mode::NEUTRAL);
        // Timeouts only count when homed — without a home reference NEUTRAL is meaningless
        const bool canTimeout  = inCanFam && canRxEver && pitchHomed && rollHomed;
        const bool cmdSmall    = canTimeout && (now2 - lastCanRx > config::CMD_TIMEOUT_SMALL_MS);
        const bool cmdLarge    = (mode == Mode::NEUTRAL) ||
                                 (canTimeout && (now2 - lastCanRx > config::CMD_TIMEOUT_LARGE_MS));

        const bool bmsEverSeen = USE_BMS && bmsManager.telemetry().any_response;
        const BmsManager::Flags& bmsF = bmsManager.flags();

        SafetyManager::Inputs sin{};
        sin.leak_raw               = leak.isLeak()       || inj_leak;
        // Ignore driver FLT for the first 3 s after boot — VMOT may still be ramping up
        // and the driver chip asserts FLT during undervoltage. After grace period the
        // debounce (500 ms) prevents transient glitches from latching a fault.
        const bool fltGrace = (now2 < 10000);  // 10s grace — covers Pi CAN initialisation
        sin.driver_fault_pitch_raw = (USE_PITCH_FLT && !fltGrace && pitch.faultActive()) || inj_driver_p;
        sin.driver_fault_roll_raw  = (!fltGrace && USE_ROLL && roll.faultActive()) || inj_driver_r;
        sin.homing_failed_pitch    = pitchHomingFailed   || inj_home_fail_p;
        sin.homing_failed_roll     = rollHomingFailed    || inj_home_fail_r;
        sin.stall_pitch_raw        = false;  // no current sensing on steppers
        sin.stall_roll_raw         = false;
        sin.cmd_timeout_small      = cmdSmall                                     || inj_cmd_s;
        sin.cmd_timeout_large      = cmdLarge                                     || inj_cmd_h;
        sin.bms_timeout_small      = (bmsEverSeen && bmsF.bms_timeout_small)      || inj_bms_to_s;
        sin.bms_timeout_large      = (bmsEverSeen && bmsF.bms_timeout_large)      || inj_bms_to_h;
        sin.bms_temp_high          = (bmsEverSeen && bmsF.bms_temp_high)          || inj_bms_temp;
        sin.bms_overcurrent        = (bmsEverSeen && bmsF.bms_overcurrent)        || inj_bms_oc;
        sin.bms_switch_fault       = (bmsEverSeen && bmsF.bms_switch_fault)       || inj_bms_sw;
        sin.bms_low_voltage        = (bmsEverSeen && bmsF.bms_low_voltage)        || inj_bms_low;
        sin.bms_high_voltage       = (bmsEverSeen && bmsF.bms_high_voltage)       || inj_bms_high;
        sin.tof_valid              = true;  // ToF not used on P&R

        lastSafety = safety.update(sin, dt);
    }

    // ----------------------------------------------------------------
    // Motor enable / disable in response to faults
    // Allow motors in NEUTRAL when the only fault is CMD_TO_L
    // (that fault is what caused NEUTRAL; motor must still move to zero).
    // ----------------------------------------------------------------
    {
        const bool onlyCmdTo = (lastSafety.hard_fault_a == can::FAULT_CMD_TO_L) &&
                               (lastSafety.hard_fault_b == 0);
        const bool suppress  = (mode == Mode::NEUTRAL) && onlyCmdTo;

        if (lastSafety.hard_fault && !suppress) {
            if (motorsEnabled) {
                pitch.disable();
                if (USE_ROLL) roll.disable();
                motorsEnabled = false;
                pitchIdleOff = false;  // fault disable takes precedence
                rollIdleOff  = false;
            }
        } else if (!motorsEnabled) {
            pitch.enable();
            if (USE_ROLL) roll.enable();
            pitchIdleOff = false;
            rollIdleOff  = false;
            lastPitchMoveMs = millis();
            lastRollMoveMs  = millis();
            motorsEnabled = true;
        }
    }

    // ----------------------------------------------------------------
    // Idle auto-disable: cut driver current when no move has occurred
    // for IDLE_DISABLE_MS. Re-enabled automatically at next move start.
    // Skipped if a hard fault already holds the driver off.
    // ----------------------------------------------------------------
    if (AUTO_DISABLE_IDLE && motorsEnabled && !lastSafety.hard_fault) {
        const uint32_t now3 = millis();
        if (!pitchIdleOff && (now3 - lastPitchMoveMs > IDLE_DISABLE_MS)) {
            pitch.disable();
            pitchIdleOff = true;
        }
        if (USE_ROLL && !rollIdleOff && (now3 - lastRollMoveMs > IDLE_DISABLE_MS)) {
            roll.disable();
            rollIdleOff = true;
        }
    }
    if (!AUTO_DISABLE_IDLE) {
        // Feature disabled at compile time — ensure drivers are awake
        if (pitchIdleOff) { pitch.enable(); pitchIdleOff = false; }
        if (rollIdleOff)  { roll.enable();  rollIdleOff  = false; }
    }

    // ----------------------------------------------------------------
    // CAN TX
    // ----------------------------------------------------------------
    if (now - lastTxControl >= CAN_TX_CONTROL_INTERVAL_MS) {
        lastTxControl = now;
        sendStatusControl();
    }
    if (now - lastTxFault >= CAN_TX_FAULT_INTERVAL_MS) {
        lastTxFault = now;
        sendStatusFault();
    }
    if (USE_BMS && (now - lastTxBms >= CAN_TX_BMS_INTERVAL_MS)) {
        lastTxBms = now;
        sendStatusBms();
        sendStatusBmsMore();
    }

    // ----------------------------------------------------------------
    // Adaptive background status print — always on.
    // 1s when moving or expecting movement, 5s when fully idle.
    {
        const bool recentMove   = (now - lastPitchMoveMs < 3000) ||
                                  (USE_ROLL && now - lastRollMoveMs < 3000);
        const bool activeMode   = (mode == Mode::HOMING) ||
                                  (mode == Mode::NEUTRAL) ||
                                  newSetpointPending;
        const uint32_t interval = (recentMove || activeMode) ? 1000 : 5000;
        if (now - lastPrint >= interval) {
            lastPrint = now;
            printStatus();
        }
    }

    // debugPrint kept for CAN frame tracing — unrelated to status cadence
}

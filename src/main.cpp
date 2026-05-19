#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "config/constants.hpp"
#include "config/pin_map.hpp"
#include "drivers/hall_sensor.hpp"
#include "drivers/push_switch.hpp"
#include "drivers/stepper_driver.hpp"

// ============================================================
// HOMING TEST
// ============================================================
// Paste this into src/main.cpp to test homing independently.
//
// What this does:
//   - Prints FWD limit / BWD limit / Hall sensor state every 1s
//     so you can verify sensor wiring before running any motors.
//   - Runs pitch or roll homing on command with verbose phase logs.
//   - Reports final position in mm / deg after homing.
//
// Roll homing logic:
//   R1: CCW until hall active  -> -60 deg magnet (limitCCW)
//   R2: CW, exit magnet, sweep -> +60 deg magnet (limitCW)
//   R3: move to midpoint (limitCCW+limitCW)/2, zero counter
//   Expects ~120 deg span between the two magnets.
//
// Commands:
//   sensors           print sensor states once
//   live on/off       toggle 1 Hz auto-print
//   homep/homer/home  run homing sequences
//   pf <mm>           pitch forward N mm  (FWD-limit protected)
//   pb <mm>           pitch backward N mm (BWD-limit protected)
//   rcw <deg>         roll CW N degrees
//   rccw <deg>        roll CCW N degrees
//   pen/pdis          enable / disable pitch driver
//   ren/rdis          enable / disable roll driver
//   status            position + sensor snapshot
//   help              this message
// ============================================================

static constexpr StepperDriver::Direction PITCH_FORWARD_DIR = StepperDriver::Direction::CCW;
static constexpr StepperDriver::Direction ROLL_CW_DIR       = StepperDriver::Direction::CW;

// Pitch runs slower than roll — lower speed keeps the motor in its
// high-torque region by limiting back-EMF, important for driving the
// lead screw against the battery pack mass.
// 150 steps/s caused driver OTP (overtemperature) fault — full current
// held too long per step. 300 steps/s is a better balance: still strong
// enough torque while reducing thermal load per phase.
static constexpr float    PITCH_HOME_SPEED     = 600.0f;  // steps/s
static constexpr float    PITCH_JOG_SPEED      = 200.0f;  // steps/s
static constexpr float    ROLL_HOME_SPEED      = 700.0f;  // steps/s  (roll load is lighter)
static constexpr float    ROLL_JOG_SPEED       = 400.0f;  // steps/s
static constexpr uint32_t PITCH_HOME_MAX_STEPS = 30000;
static constexpr uint32_t ROLL_HOME_MAX_STEPS  = 15000;

static StepperDriver pitch({
    .pin_step       = PIN_PITCH_STEP,
    .pin_dir        = PIN_PITCH_DIR,
    .pin_en         = PIN_PITCH_ENA,
    .pin_flt        = PIN_PITCH_FLT,
    .step_pulse_us  = 5.0f,
    .steps_per_sec  = PITCH_HOME_SPEED,
    .flt_active_low = true,
    .en_active_low  = true
});

static StepperDriver roll({
    .pin_step       = PIN_ROLL_STEP,
    .pin_dir        = PIN_ROLL_DIR,
    .pin_en         = PIN_ROLL_ENA,
    .pin_flt        = PIN_ROLL_FLT,
    .step_pulse_us  = 5.0f,
    .steps_per_sec  = ROLL_HOME_SPEED,
    .flt_active_low = true,
    .en_active_low  = true
});

static PushSwitch pitchFwdLimit({ .pin = PIN_FWD_SWH,   .active_low = true, .use_pullup = true });
static PushSwitch pitchRevLimit({ .pin = PIN_BWD_SWH,   .active_low = true, .use_pullup = true });
static HallSensor rollHall     ({ .pin = PIN_HALL_SENS, .active_low = true, .use_pullup = true });

static int32_t pitchSteps = 0;
static int32_t rollSteps  = 0;
static bool    pitchHomed = false;
static bool    rollHomed  = false;
static bool    liveMode   = true;   // 1 Hz auto-print on by default
static String  rx;

static uint32_t lastLivePrint = 0;

// LED heartbeat
static uint32_t lastLedToggle = 0;
static bool     ledOn         = false;

// ----------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------
static StepperDriver::Direction opposite(StepperDriver::Direction d) {
    return d == StepperDriver::Direction::CW
        ? StepperDriver::Direction::CCW : StepperDriver::Direction::CW;
}

static float pitchStepsToMm(int32_t s) { return s * config::PITCH_STEP_MM; }
static float rollStepsToDeg(int32_t s) { return s * config::ROLL_STEP_DEG; }

static uint32_t absI32(int32_t v) {
    return v >= 0 ? (uint32_t)v : (uint32_t)(-v);
}

// ----------------------------------------------------------------
// Driver reset helpers
// ----------------------------------------------------------------
// DRV8825 latches a fault (OCP / OTP) on the FLT pin and stops
// outputting current. Toggling the EN pin clears the latch.
// Call between homing phases and before every jog to ensure a
// clean slate even if a previous move left the driver faulted.
static void resetPitchDriver() {
    pitch.disable();
    delay(300);   // allow chip to cool and latch to clear
    pitch.enable();
    delay(10);
    if (pitch.faultActive()) {
        Serial.println("[drv] WARN: pitch fault still active after reset");
    }
}

static void resetRollDriver() {
    roll.disable();
    delay(100);
    roll.enable();
    delay(10);
    if (roll.faultActive()) {
        Serial.println("[drv] WARN: roll fault still active after reset");
    }
}

// ----------------------------------------------------------------
// Jog helpers
// ----------------------------------------------------------------
// jogPitch: distance in mm (positive = forward, negative = backward).
// Converts to steps internally. Stops early on limit switch or driver fault.
static void jogPitch(float mm) {
    if (mm == 0.0f) return;
    const int32_t steps = (int32_t)roundf(mm / config::PITCH_STEP_MM);
    if (steps == 0) return;

    // Warn if position tracking is already unreliable.
    if (!pitchHomed) {
        Serial.println("[jog] WARN: pitch not homed — step counter may not reflect real position");
    }

    resetPitchDriver();
    pitch.setSpeed(PITCH_JOG_SPEED);
    const bool fwd = steps > 0;
    pitch.setDirection(fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
    const uint32_t n = absI32(steps);
    uint32_t moved = 0;
    bool faulted = false;
    for (uint32_t i = 0; i < n; i++) {
        if (pitch.faultActive()) {
            Serial.println("[jog] ABORT pitch: driver fault");
            Serial.println("      POSITION LOST — step counter unreliable, rehome required");
            pitchHomed = false;   // position is no longer trusted
            faulted = true;
            break;
        }
        if (fwd  && pitchFwdLimit.isPressed()) { Serial.println("[jog] STOP pitch: FWD limit hit"); break; }
        if (!fwd && pitchRevLimit.isPressed()) { Serial.println("[jog] STOP pitch: BWD limit hit"); break; }
        pitch.step();
        pitchSteps += fwd ? 1 : -1;
        moved++;
    }
    Serial.print("[jog] pitch ");
    Serial.print(fwd ? "+" : "-");
    Serial.print(moved * config::PITCH_STEP_MM, 2);
    Serial.print(" mm");
    if (!faulted) {
        Serial.print("  pos=");
        Serial.print(pitchStepsToMm(pitchSteps), 3);
        Serial.print(" mm");
    } else {
        Serial.print("  pos=UNKNOWN");
    }
    Serial.println();
}

// jogRoll: angle in degrees (positive = CW, negative = CCW).
// Converts to steps internally.
static void jogRoll(float deg) {
    if (deg == 0.0f) return;
    const int32_t steps = (int32_t)roundf(deg / config::ROLL_STEP_DEG);
    if (steps == 0) return;
    resetRollDriver();
    roll.setSpeed(ROLL_JOG_SPEED);
    const bool cw = steps > 0;
    roll.setDirection(cw ? ROLL_CW_DIR : opposite(ROLL_CW_DIR));
    const uint32_t n = absI32(steps);
    uint32_t moved = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (roll.faultActive()) { Serial.println("[jog] ABORT roll: driver fault"); break; }
        roll.step();
        rollSteps += cw ? 1 : -1;
        moved++;
    }
    Serial.print("[jog] roll ");
    Serial.print(cw ? "+" : "-");
    Serial.print(moved * config::ROLL_STEP_DEG, 2);
    Serial.print(" deg  pos=");
    Serial.print(rollStepsToDeg(rollSteps), 3);
    Serial.println(" deg");
}

// ----------------------------------------------------------------
// Sensor print (called at 1 Hz and on demand)
// ----------------------------------------------------------------
static void printSensors() {
    Serial.print("[sensors]  FWD=");
    Serial.print(pitchFwdLimit.isPressed() ? "PRESSED" : "open   ");
    Serial.print("  BWD=");
    Serial.print(pitchRevLimit.isPressed() ? "PRESSED" : "open   ");
    Serial.print("  HALL=");
    Serial.print(rollHall.isActive() ? "ACTIVE " : "inactive");
    Serial.print("  flt_P=");
    Serial.print(pitch.faultActive() ? "FAULT" : "ok");
    Serial.print("  flt_R=");
    Serial.println(roll.faultActive() ? "FAULT" : "ok");
}

static void printStatus() {
    Serial.println();
    Serial.print("homed_P="); Serial.print(pitchHomed ? "YES" : "NO (REHOME REQUIRED)");
    Serial.print("  pitch=");
    if (pitchHomed) {
        Serial.print(pitchStepsToMm(pitchSteps), 3);
        Serial.print(" mm  ("); Serial.print(pitchSteps); Serial.print(" steps)");
    } else {
        Serial.print("UNKNOWN");
    }
    Serial.println();

    Serial.print("homed_R="); Serial.print(rollHomed ? "YES" : "NO");
    Serial.print("  roll=");  Serial.print(rollStepsToDeg(rollSteps), 3);
    Serial.print(" deg  ("); Serial.print(rollSteps); Serial.println(" steps)");

    printSensors();
}

static void printHelp() {
    Serial.println();
    Serial.println("=== HOMING TEST ===");
    Serial.println("sensors       : print sensor states once");
    Serial.println("live on/off   : toggle 1 Hz auto sensor print");
    Serial.println("--- Homing ---");
    Serial.println("homep         : run pitch homing");
    Serial.println("homer         : run roll homing");
    Serial.println("home          : run pitch + roll homing");
    Serial.println("status        : position + sensor snapshot");
    Serial.println("--- Jog ---");
    Serial.println("pf <mm>       : pitch forward  (e.g. pf 5.0)  stops at FWD limit");
    Serial.println("pb <mm>       : pitch backward (e.g. pb 2.5)  stops at BWD limit");
    Serial.println("rcw <deg>     : roll CW        (e.g. rcw 30)");
    Serial.println("rccw <deg>    : roll CCW       (e.g. rccw 15)");
    Serial.println("--- Motor enable ---");
    Serial.println("pen / pdis    : enable / disable pitch driver");
    Serial.println("ren / rdis    : enable / disable roll driver");
    Serial.println("help          : this message");
    Serial.println();
    Serial.println("Tip: watch the 1 Hz line while manually pressing");
    Serial.println("     the switches / moving the roll mechanism");
    Serial.println("     to verify sensors read correctly BEFORE homing.");
    Serial.println();
}

// ----------------------------------------------------------------
// Pitch homing
// ----------------------------------------------------------------
static bool homePitch() {
    Serial.println();
    Serial.println(">>> PITCH HOMING START");
    pitchHomed = false;
    bool ok = false;

    pitch.setSpeed(PITCH_HOME_SPEED);

    do {
        // ----------------------------------------------------------
        // Phase 1: move FORWARD until FWD limit switch presses.
        // ----------------------------------------------------------
        Serial.println("[P1] seeking FWD limit switch...");
        pitch.setDirection(PITCH_FORWARD_DIR);

        uint32_t moved = 0;
        while (!pitchFwdLimit.isPressed() && moved < PITCH_HOME_MAX_STEPS) {
            if (pitch.faultActive()) {
                Serial.println("[P1] ABORT: driver fault");
                break;
            }
            pitch.step();
            pitchSteps++;
            moved++;
        }

        if (!pitchFwdLimit.isPressed()) {
            Serial.print("[P1] FAILED after "); Serial.print(moved);
            Serial.println(" steps — FWD limit not reached");
            break;
        }
        const int32_t fwdHit = pitchSteps;
        Serial.print("[P1] FWD limit hit at step "); Serial.println(fwdHit);

        // Reset driver between phases — clears any thermal latch from P1.
        Serial.println("[P1] resetting driver before reversal...");
        resetPitchDriver();
        if (pitch.faultActive()) { Serial.println("[P1] ABORT: fault persists after reset"); break; }

        // ----------------------------------------------------------
        // Phase 2: move BACKWARD until BWD limit switch presses.
        // ----------------------------------------------------------
        Serial.println("[P2] seeking BWD limit switch...");
        pitch.setDirection(opposite(PITCH_FORWARD_DIR));

        moved = 0;
        while (!pitchRevLimit.isPressed() && moved < PITCH_HOME_MAX_STEPS) {
            if (pitch.faultActive()) {
                Serial.println("[P2] ABORT: driver fault");
                break;
            }
            pitch.step();
            pitchSteps--;
            moved++;
        }

        if (!pitchRevLimit.isPressed()) {
            Serial.print("[P2] FAILED after "); Serial.print(moved);
            Serial.println(" steps — BWD limit not reached");
            break;
        }
        const int32_t revHit = pitchSteps;
        Serial.print("[P2] BWD limit hit at step "); Serial.println(revHit);

        // Reset driver again before centering move.
        Serial.println("[P2] resetting driver before centering...");
        resetPitchDriver();
        if (pitch.faultActive()) { Serial.println("[P2] ABORT: fault persists after reset"); break; }

        const int32_t span = fwdHit - revHit;
        Serial.print("[P2] Span = "); Serial.print(span);
        Serial.print(" steps = "); Serial.print(span * config::PITCH_STEP_MM, 2);
        Serial.println(" mm");

        if (span <= 0) {
            Serial.println("[P2] FAILED: span invalid (FWD hit <= BWD hit)");
            break;
        }

        // ----------------------------------------------------------
        // Phase 3: move to center, zero the counter.
        // ----------------------------------------------------------
        const int32_t center = revHit + (span / 2);
        const int32_t delta  = center - pitchSteps;
        Serial.print("[P3] moving to center at step "); Serial.println(center);

        if (delta != 0) {
            const bool fwd = delta > 0;
            pitch.setDirection(fwd ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR));
            const uint32_t n = absI32(delta);
            for (uint32_t i = 0; i < n; i++) {
                if (pitch.faultActive()) {
                    Serial.println("[P3] ABORT: driver fault while centering");
                    break;
                }
                if (fwd  && pitchFwdLimit.isPressed()) { Serial.println("[P3] ABORT: FWD limit hit while centering"); break; }
                if (!fwd && pitchRevLimit.isPressed()) { Serial.println("[P3] ABORT: BWD limit hit while centering"); break; }
                pitch.step();
                pitchSteps += fwd ? 1 : -1;
            }
            if (pitch.faultActive()) break;
            if (pitchFwdLimit.isPressed() || pitchRevLimit.isPressed()) break;
        }

        pitchSteps = 0;
        pitch.setDirection(PITCH_FORWARD_DIR);

        // ----------------------------------------------------------
        // Sanity check: at claimed centre neither limit should be
        // pressed. If BWD is still pressed the motor never left the
        // end-stop — it stalled silently during the centering move.
        // This is the key guard against open-loop position corruption.
        // ----------------------------------------------------------
        if (pitchFwdLimit.isPressed()) {
            Serial.println("[P3] SANITY FAIL: FWD limit still pressed at claimed centre");
            Serial.println("     Motor likely stalled during centering. REHOME.");
        } else if (pitchRevLimit.isPressed()) {
            Serial.println("[P3] SANITY FAIL: BWD limit still pressed at claimed centre");
            Serial.println("     Motor did not leave end-stop — stall during centering. REHOME.");
        } else {
            pitchHomed = true;
            ok = true;
        }

    } while (false);

    if (ok) {
        Serial.println(">>> PITCH HOMING OK — position zeroed at center");
    } else {
        pitchHomed = false;  // guarantee bad state is never silently kept
        Serial.println(">>> PITCH HOMING FAILED — position unknown, rehome required");
    }
    return ok;
}

// ----------------------------------------------------------------
// Roll homing
// ----------------------------------------------------------------
static bool homeRoll() {
    Serial.println();
    Serial.println(">>> ROLL HOMING START");
    Serial.println("Assumes mechanism starts near centre (not on a magnet).");
    rollHomed = false;
    bool ok = false;

    roll.setSpeed(ROLL_HOME_SPEED);

    do {
        uint32_t moved = 0;

        // ----------------------------------------------------------
        // Phase 1: CCW until hall active — finds the -60 deg magnet.
        // ----------------------------------------------------------
        Serial.println("[R1] CCW seeking -60 magnet...");
        roll.setDirection(opposite(ROLL_CW_DIR));

        while (!rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (roll.faultActive()) { Serial.println("[R1] ABORT: driver fault"); break; }
            roll.step();
            rollSteps--;
            moved++;
        }

        if (!rollHall.isActive()) {
            Serial.print("[R1] FAILED after "); Serial.print(moved);
            Serial.println(" steps — -60 magnet not found");
            break;
        }
        const int32_t limitCCW = rollSteps;
        Serial.print("[R1] -60 magnet found at step "); Serial.print(limitCCW);
        Serial.print(" ("); Serial.print(limitCCW * config::ROLL_STEP_DEG, 2); Serial.println(" deg)");

        // ----------------------------------------------------------
        // Phase 2: CW — exit the -60 magnet, sweep to +60 magnet.
        // ----------------------------------------------------------
        Serial.println("[R2] CW — exiting -60 magnet then seeking +60 magnet...");
        roll.setDirection(ROLL_CW_DIR);
        moved = 0;

        // exit the -60 magnet first
        while (rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (roll.faultActive()) { Serial.println("[R2] ABORT: driver fault"); break; }
            roll.step();
            rollSteps++;
            moved++;
        }
        if (rollHall.isActive()) {
            Serial.println("[R2] FAILED: could not exit -60 magnet");
            break;
        }
        Serial.println("[R2] Exited -60 magnet, continuing CW...");

        // sweep CW to the +60 magnet
        while (!rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            if (roll.faultActive()) { Serial.println("[R2] ABORT: driver fault"); break; }
            roll.step();
            rollSteps++;
            moved++;
        }
        if (!rollHall.isActive()) {
            Serial.print("[R2] FAILED after "); Serial.print(moved);
            Serial.println(" steps — +60 magnet not found");
            break;
        }
        const int32_t limitCW = rollSteps;
        Serial.print("[R2] +60 magnet found at step "); Serial.print(limitCW);
        Serial.print(" ("); Serial.print(limitCW * config::ROLL_STEP_DEG, 2); Serial.println(" deg)");

        const int32_t span = limitCW - limitCCW;
        Serial.print("[R2] Span = "); Serial.print(span);
        Serial.print(" steps = "); Serial.print(span * config::ROLL_STEP_DEG, 2);
        Serial.println(" deg  (expect ~120)");

        // ----------------------------------------------------------
        // Phase 3: move to midpoint between the two magnets, zero.
        // ----------------------------------------------------------
        const int32_t center = (limitCCW + limitCW) / 2;
        const int32_t delta  = center - rollSteps;
        Serial.print("[R3] Moving to midpoint step "); Serial.println(center);

        if (delta != 0) {
            const bool cw = delta > 0;
            roll.setDirection(cw ? ROLL_CW_DIR : opposite(ROLL_CW_DIR));
            const uint32_t n = absI32(delta);
            for (uint32_t i = 0; i < n; i++) {
                if (roll.faultActive()) {
                    Serial.println("[R3] ABORT: driver fault while centering");
                    break;
                }
                roll.step();
                rollSteps += cw ? 1 : -1;
            }
            if (roll.faultActive()) break;
        }

        rollSteps = 0;
        roll.setDirection(ROLL_CW_DIR);
        rollHomed = true;
        ok = true;

    } while (false);

    if (ok) {
        Serial.println(">>> ROLL HOMING OK — midpoint between magnets zeroed");
    } else {
        Serial.println(">>> ROLL HOMING FAILED");
    }
    return ok;
}

// ----------------------------------------------------------------
// Serial command handler
// ----------------------------------------------------------------
static void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    if (cmd == "help")     { printHelp(); return; }
    if (cmd == "status")   { printStatus(); return; }
    if (cmd == "sensors")  { printSensors(); return; }
    if (cmd == "live on")  { liveMode = true;  Serial.println("Live print ON");  return; }
    if (cmd == "live off") { liveMode = false; Serial.println("Live print OFF"); return; }

    if (cmd == "homep") {
        homePitch();
        printStatus();
        return;
    }
    if (cmd == "homer") {
        homeRoll();
        printStatus();
        return;
    }
    if (cmd == "home") {
        bool p = homePitch();
        bool r = homeRoll();
        Serial.print("Home summary: pitch="); Serial.print(p ? "OK" : "FAIL");
        Serial.print("  roll="); Serial.println(r ? "OK" : "FAIL");
        printStatus();
        return;
    }

    if (cmd == "pen")  { pitch.enable();  Serial.println("Pitch enabled");  return; }
    if (cmd == "pdis") { pitch.disable(); Serial.println("Pitch disabled"); return; }
    if (cmd == "ren")  { roll.enable();   Serial.println("Roll enabled");   return; }
    if (cmd == "rdis") { roll.disable();  Serial.println("Roll disabled");  return; }

    if (cmd.startsWith("pf ")) {
        float mm = cmd.substring(3).toFloat();
        if (mm <= 0.0f) { Serial.println("Usage: pf <mm>  e.g. pf 5.0"); return; }
        jogPitch(mm);
        return;
    }
    if (cmd.startsWith("pb ")) {
        float mm = cmd.substring(3).toFloat();
        if (mm <= 0.0f) { Serial.println("Usage: pb <mm>  e.g. pb 2.5"); return; }
        jogPitch(-mm);
        return;
    }
    if (cmd.startsWith("rcw ")) {
        float deg = cmd.substring(4).toFloat();
        if (deg <= 0.0f) { Serial.println("Usage: rcw <deg>  e.g. rcw 30"); return; }
        jogRoll(deg);
        return;
    }
    if (cmd.startsWith("rccw ")) {
        float deg = cmd.substring(5).toFloat();
        if (deg <= 0.0f) { Serial.println("Usage: rccw <deg>  e.g. rccw 15"); return; }
        jogRoll(-deg);
        return;
    }

    Serial.print("Unknown: "); Serial.println(cmd);
}

static void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (rx.length() > 0) { handleCommand(rx); rx = ""; }
        } else {
            rx += c;
        }
    }
}

// ----------------------------------------------------------------
// Setup / Loop
// ----------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pitch.begin();
    roll.begin();
    pitchFwdLimit.begin();
    pitchRevLimit.begin();
    rollHall.begin();

    pitch.enable();
    roll.enable();

    printHelp();
    Serial.println("Live sensor readings active (1 Hz). Manually trigger");
    Serial.println("each sensor now to confirm they read correctly.");
    Serial.println("Then type 'homep', 'homer', or 'home' to test homing.");
    Serial.println();
}

void loop() {
    const uint32_t now = millis();

    // LED heartbeat: 100 ms flash every 5 s
    if (!ledOn && (now - lastLedToggle >= 5000)) {
        digitalWrite(LED_BUILTIN, HIGH);
        ledOn = true;
        lastLedToggle = now;
    }
    if (ledOn && (now - lastLedToggle >= 100)) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOn = false;
    }

    readSerial();

    if (liveMode && now - lastLivePrint >= 1000) {
        lastLivePrint = now;
        printSensors();
    }
}

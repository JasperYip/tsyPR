#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "config/constants.hpp"
#include "config/pin_map.hpp"
#include "drivers/hall_sensor.hpp"
#include "drivers/push_switch.hpp"
#include "drivers/stepper_driver.hpp"

// Paste this into src/main.cpp for manual direction checks.
// Goal:
// 1 Verify pitch forward/backward wiring
// 2 Verify roll CW/CCW wiring
//
// If a direction is reversed physically, only change these two constants.
static constexpr StepperDriver::Direction PITCH_FORWARD_DIR = StepperDriver::Direction::CW;
static constexpr StepperDriver::Direction ROLL_CW_DIR = StepperDriver::Direction::CW;
static constexpr float PITCH_HOME_SPEED_STEPS_S = 700.0f;
static constexpr float ROLL_HOME_SPEED_STEPS_S = 700.0f;
static constexpr uint32_t PITCH_HOME_MAX_STEPS = 30000;
static constexpr uint32_t ROLL_HOME_MAX_STEPS = 15000;

StepperDriver pitch({
    .pin_step = PIN_PITCH_STEP,
    .pin_dir = PIN_PITCH_DIR,
    .pin_en = PIN_PITCH_ENA,
    .pin_flt = PIN_PITCH_FLT,
    .step_pulse_us = 5.0f,
    .steps_per_sec = 1200.0f,
    .flt_active_low = true,
    .en_active_low = true
});

StepperDriver roll({
    .pin_step = PIN_ROLL_STEP,
    .pin_dir = PIN_ROLL_DIR,
    .pin_en = PIN_ROLL_ENA,
    .pin_flt = PIN_ROLL_FLT,
    .step_pulse_us = 5.0f,
    .steps_per_sec = 1200.0f,
    .flt_active_low = true,
    .en_active_low = true
});

StepperDriver::Direction pitchDir = PITCH_FORWARD_DIR;
StepperDriver::Direction rollDir = ROLL_CW_DIR;

int32_t pitchSteps = 0;
int32_t rollSteps = 0;

PushSwitch pitchFwdLimit({ .pin = PIN_FWD_SWH, .active_low = true, .use_pullup = true });
PushSwitch pitchRevLimit({ .pin = PIN_BWD_SWH, .active_low = true, .use_pullup = true });
HallSensor rollHall({ .pin = PIN_HALL_SENS, .active_low = true, .use_pullup = true });

bool motorsEnabled = true;
bool autoModeEnabled = false;
bool pitchHomed = false;
bool rollHomed = false;
String rx;

int32_t pitchMmToSteps(float mm) {
    return static_cast<int32_t>(lroundf(mm / config::PITCH_STEP_MM));
}

int32_t rollDegToSteps(float deg) {
    return static_cast<int32_t>(lroundf(deg / config::ROLL_STEP_DEG));
}

float pitchStepsToMm(int32_t steps) {
    return static_cast<float>(steps) * config::PITCH_STEP_MM;
}

float rollStepsToDeg(int32_t steps) {
    return static_cast<float>(steps) * config::ROLL_STEP_DEG;
}

StepperDriver::Direction opposite(StepperDriver::Direction d) {
    return (d == StepperDriver::Direction::CW)
        ? StepperDriver::Direction::CCW
        : StepperDriver::Direction::CW;
}

float clampf(float v, float minV, float maxV) {
    if (v < minV) return minV;
    if (v > maxV) return maxV;
    return v;
}

bool parseFloatStrict(const String& text, float& outValue) {
    String token = text;
    token.trim();
    if (token.length() == 0) {
        return false;
    }

    const char* begin = token.c_str();
    char* end = nullptr;
    outValue = strtof(begin, &end);
    if (end == begin) {
        return false;
    }

    while (*end == ' ' || *end == '\t') {
        ++end;
    }

    return (*end == '\0') && isfinite(outValue);
}

bool homePitch();
bool homeRoll();
void movePitchToMmAbs(float targetMm);
void moveRollToDegAbs(float targetDeg);

void printHelp() {
    Serial.println();
    Serial.println("=== 2 MOTOR MANUAL DIRECTION TEST ===");
    Serial.println("help        : print commands");
    Serial.println("status      : show current status");
    Serial.println("homep       : run pitch homing (fwd sw -> rev sw -> center)");
    Serial.println("homer       : run roll homing (hall edge center)");
    Serial.println("home        : run pitch + roll homing");
    Serial.println("auto on     : enable auto mode (requires both axes homed)");
    Serial.println("auto off    : disable auto mode");
    Serial.println("en          : enable both drivers");
    Serial.println("dis         : disable both drivers");
    Serial.println("zero        : reset software position counters");
    Serial.println();
    Serial.println("Pitch:");
    Serial.println("pf          : set pitch direction Forward");
    Serial.println("pb          : set pitch direction Backward");
    Serial.println("pm <mm>     : move pitch by <mm> in selected direction");
    Serial.println("ps <steps>  : move pitch by <steps> in selected direction");
    Serial.println("ap <mm>     : AUTO pitch absolute setpoint (mm)");
    Serial.println();
    Serial.println("Roll:");
    Serial.println("rcw         : set roll direction CW");
    Serial.println("rccw        : set roll direction CCW");
    Serial.println("rm <deg>    : move roll by <deg> in selected direction");
    Serial.println("rs <steps>  : move roll by <steps> in selected direction");
    Serial.println("ar <deg>    : AUTO roll absolute setpoint (deg)");
    Serial.println("goto <mm> <deg> : AUTO absolute pitch+roll setpoint");
    Serial.println();
}

void printStatus() {
    Serial.println();
    Serial.print("motors=");
    Serial.println(motorsEnabled ? "EN" : "DIS");
    Serial.print("mode=");
    Serial.println(autoModeEnabled ? "AUTO" : "MANUAL");
    Serial.print("homed_pitch=");
    Serial.print(pitchHomed ? "YES" : "NO");
    Serial.print("  homed_roll=");
    Serial.println(rollHomed ? "YES" : "NO");

    Serial.print("pitch_dir=");
    Serial.print((pitchDir == PITCH_FORWARD_DIR) ? "FWD" : "BWD");
    Serial.print("  pitch_mm=");
    Serial.print(pitchStepsToMm(pitchSteps), 3);
    Serial.print("  pitch_steps=");
    Serial.print(pitchSteps);
    Serial.print("  pitch_fault=");
    Serial.print(pitch.faultActive() ? "YES" : "NO");
    Serial.print("  lim_fwd=");
    Serial.print(pitchFwdLimit.isPressed() ? "PRESSED" : "OPEN");
    Serial.print("  lim_rev=");
    Serial.println(pitchRevLimit.isPressed() ? "PRESSED" : "OPEN");

    Serial.print("roll_dir=");
    Serial.print((rollDir == ROLL_CW_DIR) ? "CW" : "CCW");
    Serial.print("  roll_deg=");
    Serial.print(rollStepsToDeg(rollSteps), 3);
    Serial.print("  roll_steps=");
    Serial.print(rollSteps);
    Serial.print("  roll_fault=");
    Serial.print(roll.faultActive() ? "YES" : "NO");
    Serial.print("  hall=");
    Serial.println(rollHall.isActive() ? "ACTIVE" : "INACTIVE");
}

void movePitchSteps(uint32_t steps) {
    if (!motorsEnabled) {
        Serial.println("Pitch blocked: drivers are disabled");
        return;
    }
    if (steps == 0) {
        Serial.println("Pitch move ignored (0 steps)");
        return;
    }

    pitch.move(steps);
    pitchSteps += (pitchDir == PITCH_FORWARD_DIR)
        ? static_cast<int32_t>(steps)
        : -static_cast<int32_t>(steps);
}

void moveRollSteps(uint32_t steps) {
    if (!motorsEnabled) {
        Serial.println("Roll blocked: drivers are disabled");
        return;
    }
    if (steps == 0) {
        Serial.println("Roll move ignored (0 steps)");
        return;
    }

    roll.move(steps);
    rollSteps += (rollDir == ROLL_CW_DIR)
        ? static_cast<int32_t>(steps)
        : -static_cast<int32_t>(steps);
}

uint32_t absStepsI32(int32_t v) {
    return (v >= 0) ? static_cast<uint32_t>(v) : static_cast<uint32_t>(-v);
}

void movePitchToMmAbs(float targetMm) {
    if (!autoModeEnabled) {
        Serial.println("Pitch absolute move blocked: AUTO mode is off");
        return;
    }
    if (!motorsEnabled) {
        Serial.println("Pitch absolute move blocked: drivers are disabled");
        return;
    }
    if (!pitchHomed) {
        Serial.println("Pitch absolute move blocked: pitch not homed");
        return;
    }

    const float clampedMm = clampf(targetMm, config::PITCH_MIN_MM, config::PITCH_MAX_MM);
    if (clampedMm != targetMm) {
        Serial.print("Pitch target clamped to ");
        Serial.print(clampedMm, 3);
        Serial.println(" mm");
    }

    const int32_t targetSteps = pitchMmToSteps(clampedMm);
    const int32_t delta = targetSteps - pitchSteps;
    if (delta == 0) {
        Serial.println("Pitch already at requested absolute target");
        return;
    }

    const bool toForward = (delta > 0);
    pitchDir = toForward ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR);
    pitch.setDirection(pitchDir);

    pitch.move(absStepsI32(delta));
    pitchSteps = targetSteps;
}

void moveRollToDegAbs(float targetDeg) {
    if (!autoModeEnabled) {
        Serial.println("Roll absolute move blocked: AUTO mode is off");
        return;
    }
    if (!motorsEnabled) {
        Serial.println("Roll absolute move blocked: drivers are disabled");
        return;
    }
    if (!rollHomed) {
        Serial.println("Roll absolute move blocked: roll not homed");
        return;
    }

    const float clampedDeg = clampf(targetDeg, config::ROLL_MIN_DEG, config::ROLL_MAX_DEG);
    if (clampedDeg != targetDeg) {
        Serial.print("Roll target clamped to ");
        Serial.print(clampedDeg, 3);
        Serial.println(" deg");
    }

    const int32_t targetSteps = rollDegToSteps(clampedDeg);
    const int32_t delta = targetSteps - rollSteps;
    if (delta == 0) {
        Serial.println("Roll already at requested absolute target");
        return;
    }

    const bool toCW = (delta > 0);
    rollDir = toCW ? ROLL_CW_DIR : opposite(ROLL_CW_DIR);
    roll.setDirection(rollDir);

    roll.move(absStepsI32(delta));
    rollSteps = targetSteps;
}

bool homePitch() {
    if (!motorsEnabled) {
        Serial.println("Pitch homing blocked: drivers are disabled");
        return false;
    }

    Serial.println("Pitch homing start...");
    bool ok = false;
    autoModeEnabled = false;
    pitchHomed = false;
    const float savedSpeed = pitch.speed();
    pitch.setSpeed(PITCH_HOME_SPEED_STEPS_S);

    do {
        // Phase 1: seek forward limit.
        pitchDir = PITCH_FORWARD_DIR;
        pitch.setDirection(pitchDir);

        uint32_t moved = 0;
        while (!pitchFwdLimit.isPressed() && moved < PITCH_HOME_MAX_STEPS) {
            pitch.step();
            pitchSteps++;
            moved++;
            if (pitch.faultActive()) {
                Serial.println("Pitch homing abort: driver fault while seeking FWD switch");
                break;
            }
        }
        if (!pitchFwdLimit.isPressed()) {
            Serial.println("Pitch homing failed: FWD limit not reached");
            break;
        }
        const int32_t fwdHit = pitchSteps;

        // Phase 2: seek reverse limit.
        pitchDir = opposite(PITCH_FORWARD_DIR);
        pitch.setDirection(pitchDir);
        moved = 0;
        while (!pitchRevLimit.isPressed() && moved < PITCH_HOME_MAX_STEPS) {
            pitch.step();
            pitchSteps--;
            moved++;
            if (pitch.faultActive()) {
                Serial.println("Pitch homing abort: driver fault while seeking REV switch");
                break;
            }
        }
        if (!pitchRevLimit.isPressed()) {
            Serial.println("Pitch homing failed: REV limit not reached");
            break;
        }
        const int32_t revHit = pitchSteps;

        const int32_t span = fwdHit - revHit;
        if (span <= 0) {
            Serial.println("Pitch homing failed: invalid span between switches");
            break;
        }

        // Phase 3: move to center.
        const int32_t center = revHit + (span / 2);
        int32_t delta = center - pitchSteps;
        if (delta != 0) {
            const bool toForward = (delta > 0);
            pitchDir = toForward ? PITCH_FORWARD_DIR : opposite(PITCH_FORWARD_DIR);
            pitch.setDirection(pitchDir);

            uint32_t steps = absStepsI32(delta);
            for (uint32_t i = 0; i < steps; i++) {
                pitch.step();
                pitchSteps += toForward ? 1 : -1;
                if (pitch.faultActive()) {
                    Serial.println("Pitch homing abort: driver fault while centering");
                    break;
                }
            }
            if (pitch.faultActive()) {
                break;
            }
        }

        // Set center as software zero.
        pitchSteps = 0;
        pitchDir = PITCH_FORWARD_DIR;
        pitch.setDirection(pitchDir);
        pitchHomed = true;
        ok = true;
    } while (false);

    pitch.setSpeed(savedSpeed);

    if (ok) {
        Serial.println("Pitch homing success: centered and zeroed");
    } else {
        Serial.println("Pitch homing ended with failure");
    }
    return ok;
}

bool homeRoll() {
    if (!motorsEnabled) {
        Serial.println("Roll homing blocked: drivers are disabled");
        return false;
    }

    Serial.println("Roll homing start...");
    bool ok = false;
    autoModeEnabled = false;
    rollHomed = false;
    const float savedSpeed = roll.speed();
    roll.setSpeed(ROLL_HOME_SPEED_STEPS_S);

    do {
        uint32_t moved = 0;

        // Phase 1: go CW until hall is inactive.
        rollDir = ROLL_CW_DIR;
        roll.setDirection(rollDir);
        while (rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            roll.step();
            rollSteps++;
            moved++;
            if (roll.faultActive()) {
                Serial.println("Roll homing abort: driver fault while clearing hall");
                break;
            }
        }
        if (rollHall.isActive()) {
            Serial.println("Roll homing failed: hall stayed active in CW clear phase");
            break;
        }

        // Phase 2: go CCW until hall becomes active.
        rollDir = opposite(ROLL_CW_DIR);
        roll.setDirection(rollDir);
        moved = 0;
        while (!rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            roll.step();
            rollSteps--;
            moved++;
            if (roll.faultActive()) {
                Serial.println("Roll homing abort: driver fault while seeking hall entry");
                break;
            }
        }
        if (!rollHall.isActive()) {
            Serial.println("Roll homing failed: hall entry not found");
            break;
        }
        const int32_t hallEnter = rollSteps;

        // Phase 3: keep CCW until hall becomes inactive again.
        moved = 0;
        while (rollHall.isActive() && moved < ROLL_HOME_MAX_STEPS) {
            roll.step();
            rollSteps--;
            moved++;
            if (roll.faultActive()) {
                Serial.println("Roll homing abort: driver fault while seeking hall exit");
                break;
            }
        }
        if (rollHall.isActive()) {
            Serial.println("Roll homing failed: hall exit not found");
            break;
        }
        const int32_t hallExit = rollSteps;

        const int32_t band = hallEnter - hallExit;
        if (band <= 0) {
            Serial.println("Roll homing failed: invalid hall band");
            break;
        }

        // Phase 4: move back to center of hall band.
        const int32_t center = hallExit + (band / 2);
        int32_t delta = center - rollSteps;
        if (delta != 0) {
            const bool toCW = (delta > 0);
            rollDir = toCW ? ROLL_CW_DIR : opposite(ROLL_CW_DIR);
            roll.setDirection(rollDir);

            uint32_t steps = absStepsI32(delta);
            for (uint32_t i = 0; i < steps; i++) {
                roll.step();
                rollSteps += toCW ? 1 : -1;
                if (roll.faultActive()) {
                    Serial.println("Roll homing abort: driver fault while centering");
                    break;
                }
            }
            if (roll.faultActive()) {
                break;
            }
        }

        // Set center as software zero.
        rollSteps = 0;
        rollDir = ROLL_CW_DIR;
        roll.setDirection(rollDir);
        rollHomed = true;
        ok = true;
    } while (false);

    roll.setSpeed(savedSpeed);

    if (ok) {
        Serial.println("Roll homing success: centered and zeroed");
    } else {
        Serial.println("Roll homing ended with failure");
    }
    return ok;
}

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    if (cmd == "help") {
        printHelp();
        return;
    }
    if (cmd == "status") {
        printStatus();
        return;
    }
    if (cmd == "auto on") {
        if (!pitchHomed || !rollHomed) {
            Serial.println("AUTO mode blocked: run homing first (home/homep+homer)");
            return;
        }
        autoModeEnabled = true;
        Serial.println("AUTO mode enabled");
        return;
    }
    if (cmd == "auto off") {
        autoModeEnabled = false;
        Serial.println("AUTO mode disabled");
        return;
    }
    if (cmd == "en") {
        pitch.enable();
        roll.enable();
        motorsEnabled = true;
        Serial.println("Drivers enabled");
        return;
    }
    if (cmd == "dis") {
        pitch.disable();
        roll.disable();
        motorsEnabled = false;
        Serial.println("Drivers disabled");
        return;
    }
    if (cmd == "zero") {
        pitchSteps = 0;
        rollSteps = 0;
        pitchHomed = false;
        rollHomed = false;
        autoModeEnabled = false;
        Serial.println("Software counters reset to zero");
        return;
    }
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
        const bool p = homePitch();
        const bool r = homeRoll();
        Serial.print("Home summary: pitch=");
        Serial.print(p ? "OK" : "FAIL");
        Serial.print(" roll=");
        Serial.println(r ? "OK" : "FAIL");
        printStatus();
        return;
    }

    // Pitch direction
    if (cmd == "pf") {
        if (autoModeEnabled) {
            Serial.println("Manual direction blocked: disable AUTO mode first");
            return;
        }
        pitchDir = PITCH_FORWARD_DIR;
        pitch.setDirection(pitchDir);
        Serial.println("Pitch direction set: FORWARD");
        return;
    }
    if (cmd == "pb") {
        if (autoModeEnabled) {
            Serial.println("Manual direction blocked: disable AUTO mode first");
            return;
        }
        pitchDir = opposite(PITCH_FORWARD_DIR);
        pitch.setDirection(pitchDir);
        Serial.println("Pitch direction set: BACKWARD");
        return;
    }

    // Roll direction
    if (cmd == "rcw") {
        if (autoModeEnabled) {
            Serial.println("Manual direction blocked: disable AUTO mode first");
            return;
        }
        rollDir = ROLL_CW_DIR;
        roll.setDirection(rollDir);
        Serial.println("Roll direction set: CW");
        return;
    }
    if (cmd == "rccw") {
        if (autoModeEnabled) {
            Serial.println("Manual direction blocked: disable AUTO mode first");
            return;
        }
        rollDir = opposite(ROLL_CW_DIR);
        roll.setDirection(rollDir);
        Serial.println("Roll direction set: CCW");
        return;
    }

    // Pitch move by mm
    if (cmd.startsWith("pm")) {
        if (autoModeEnabled) {
            Serial.println("Manual move blocked: disable AUTO mode first");
            return;
        }
        String arg = cmd.substring(2);
        float mm = 0.0f;
        if (!parseFloatStrict(arg, mm) || mm <= 0.0f) {
            Serial.println("Invalid pitch mm. Example: pm 5");
            return;
        }

        int32_t steps = pitchMmToSteps(mm);
        if (steps <= 0) {
            Serial.println("Pitch move too small after conversion");
            return;
        }

        movePitchSteps(static_cast<uint32_t>(steps));
        printStatus();
        return;
    }

    // Pitch move by steps
    if (cmd.startsWith("ps")) {
        if (autoModeEnabled) {
            Serial.println("Manual move blocked: disable AUTO mode first");
            return;
        }
        String arg = cmd.substring(2);
        arg.trim();
        long steps = arg.toInt();
        if (steps <= 0) {
            Serial.println("Invalid pitch steps. Example: ps 200");
            return;
        }

        movePitchSteps(static_cast<uint32_t>(steps));
        printStatus();
        return;
    }

    // Roll move by deg
    if (cmd.startsWith("rm")) {
        if (autoModeEnabled) {
            Serial.println("Manual move blocked: disable AUTO mode first");
            return;
        }
        String arg = cmd.substring(2);
        float deg = 0.0f;
        if (!parseFloatStrict(arg, deg) || deg <= 0.0f) {
            Serial.println("Invalid roll deg. Example: rm 5");
            return;
        }

        int32_t steps = rollDegToSteps(deg);
        if (steps <= 0) {
            Serial.println("Roll move too small after conversion");
            return;
        }

        moveRollSteps(static_cast<uint32_t>(steps));
        printStatus();
        return;
    }

    // Roll move by steps
    if (cmd.startsWith("rs")) {
        if (autoModeEnabled) {
            Serial.println("Manual move blocked: disable AUTO mode first");
            return;
        }
        String arg = cmd.substring(2);
        arg.trim();
        long steps = arg.toInt();
        if (steps <= 0) {
            Serial.println("Invalid roll steps. Example: rs 200");
            return;
        }

        moveRollSteps(static_cast<uint32_t>(steps));
        printStatus();
        return;
    }
    
    // AUTO pitch absolute
    if (cmd.startsWith("ap")) {
        String arg = cmd.substring(2);
        float mm = 0.0f;
        if (!parseFloatStrict(arg, mm)) {
            Serial.println("Invalid pitch absolute target. Example: ap 12.5");
            return;
        }
        movePitchToMmAbs(mm);
        printStatus();
        return;
    }

    // AUTO roll absolute
    if (cmd.startsWith("ar")) {
        String arg = cmd.substring(2);
        float deg = 0.0f;
        if (!parseFloatStrict(arg, deg)) {
            Serial.println("Invalid roll absolute target. Example: ar -6.0");
            return;
        }
        moveRollToDegAbs(deg);
        printStatus();
        return;
    }

    // AUTO absolute pitch + roll in one command.
    if (cmd.startsWith("goto")) {
        if (!autoModeEnabled) {
            Serial.println("goto blocked: AUTO mode is off");
            return;
        }

        String args = cmd.substring(4);
        args.trim();
        int sep = args.indexOf(' ');
        if (sep < 0) {
            Serial.println("Invalid goto format. Example: goto 12.5 -6.0");
            return;
        }

        String pStr = args.substring(0, sep);
        String rStr = args.substring(sep + 1);
        pStr.trim();
        rStr.trim();
        if (pStr.length() == 0 || rStr.length() == 0) {
            Serial.println("Invalid goto format. Example: goto 12.5 -6.0");
            return;
        }

        float pMm = 0.0f;
        float rDeg = 0.0f;
        if (!parseFloatStrict(pStr, pMm) || !parseFloatStrict(rStr, rDeg)) {
            Serial.println("Invalid goto values. Example: goto 12.5 -6.0");
            return;
        }
        movePitchToMmAbs(pMm);
        moveRollToDegAbs(rDeg);
        printStatus();
        return;
    }

    Serial.println("Unknown command. Type: help");
}

void readSerial() {
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (rx.length() > 0) {
                handleCommand(rx);
                rx = "";
            }
        } else {
            rx += c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    pitch.begin();
    roll.begin();
    pitchFwdLimit.begin();
    pitchRevLimit.begin();
    rollHall.begin();

    pitch.setSpeed(1200.0f);
    roll.setSpeed(1200.0f);

    pitch.setDirection(pitchDir);
    roll.setDirection(rollDir);

    pitch.enable();
    roll.enable();
    motorsEnabled = true;

    printHelp();
    printStatus();
}

void loop() {
    readSerial();
}

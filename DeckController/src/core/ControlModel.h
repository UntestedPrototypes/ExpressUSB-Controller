#pragma once
// The control scheme (see PLAN.md section 1): turns controller input into the normalised control values of the robot, and runs the
// arm / emergency-stop state machine. Pure C++, no Qt, no SDL.
#include "InputState.h"

namespace deck {

struct ControlConfig {
    float deadZone = 0.08f;
    float expo = 0.3f;                 // 0 = linear, 1 = cubic
    float heightRate01 = 0.5f;         // body height range per second at full stick (0.5 = 50 mm/s over the robot's 100 mm range)
    float speedMin = 0.4f;             // speed scale with R2 released
    float speedMax = 1.0f;             // ... and fully pressed
    float tiltGain = 2.0f;             // touchpad: full tilt after moving this fraction of the pad / 0.5
    float stanceGain = 2.0f;
    float stepStep01 = 0.125f;         // D-pad step height: 5 mm of the robot's 40 mm range
    float armHoldS = 2.0f;
    float resetHoldS = 2.0f;
};

// Normalised outputs; every value -1..+1 (roll, pitch, stance also: 0 = straight / default)
struct ControlOutput {
    float strafe = 0, forward = 0, height = 0, turn = 0;
    float roll = 0, pitch = 0, stance = 0;
    bool armed = false, estop = false;
    bool gaitRipple = false;
    int solver = 1;                    // 0 analytic, 1 auto, 2 numeric
    float stepHeight = 0;              // -1..+1 over the robot's range
    // for the screen
    float armProgress = 0;             // 0..1 while A is held to arm
    float resetProgress = 0;           // 0..1 while Menu is held to reset the E-stop
    bool tiltRecorded = false, stanceRecorded = false;
    bool nextPage = false;             // View was pressed (one update only)
    bool controllerLost = false;       // a controller disconnect disarmed the robot
};

class ControlModel {
public:
    explicit ControlModel(const ControlConfig& cfg = ControlConfig()) : cfg_(cfg) {}

    void setConfig(const ControlConfig& c) { cfg_ = c; }
    const ControlConfig& config() const { return cfg_; }

    // Call at a fixed rate (dt in seconds)
    ControlOutput update(const InputState& in, float dt);

    // Software events (touch screen buttons, focus loss)
    void disarm();
    void emergencyStop();

    const ControlOutput& output() const { return out_; }

private:
    static float shape(float v, float deadZone, float expo);

    ControlConfig cfg_;
    ControlOutput out_;
    InputState prev_;
    float height01_ = 0.5f;            // body height setpoint
    float armHeld_ = 0, resetHeld_ = 0;
    bool aReleasedSinceArm_ = true;
    // touchpad relative touch
    bool rightTouching_ = false, leftTouching_ = false;
    float rightStartX_ = 0, rightStartY_ = 0, leftStartY_ = 0;
    float rollTrim_ = 0, pitchTrim_ = 0, stanceTrim_ = 0;
    float rollLive_ = 0, pitchLive_ = 0, stanceLive_ = 0;
    bool ripple_ = false;
    int solver_ = 1;
    float step01_ = 0.5f;              // 0..1
};

}  // namespace deck

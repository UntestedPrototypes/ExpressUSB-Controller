#include "ControlModel.h"

#include <algorithm>
#include <cmath>

namespace deck {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline bool rising(bool now, bool before) { return now && !before; }
}  // namespace

float ControlModel::shape(float v, float dz, float expo) {
    const float a = std::fabs(v);
    if (a <= dz) return 0.0f;
    float s = (a - dz) / (1.0f - dz);                 // rescaled so the stick still reaches 1
    s = (1.0f - expo) * s + expo * s * s * s;
    return v < 0 ? -s : s;
}

void ControlModel::disarm() {
    out_.armed = false;
    armHeld_ = 0;
    aReleasedSinceArm_ = false;                        // an A still held does not start a new arm
}

void ControlModel::emergencyStop() {
    out_.estop = true;
    disarm();
}

ControlOutput ControlModel::update(const InputState& in, float dt) {
    out_.nextPage = false;
    out_.controllerLost = false;

    // --- controller presence: losing it disarms (not an E-stop)
    if (!in.connected) {
        if (prev_.connected || out_.armed) out_.controllerLost = out_.armed;
        disarm();
        out_.armProgress = out_.resetProgress = 0;
        resetHeld_ = 0;
        rightTouching_ = leftTouching_ = false;
        rollLive_ = pitchLive_ = stanceLive_ = 0;
        out_.strafe = out_.forward = out_.turn = 0;
        out_.roll = clampf(rollTrim_, -1, 1);
        out_.pitch = clampf(pitchTrim_, -1, 1);
        out_.stance = clampf(stanceTrim_, -1, 1);
        prev_ = in;
        return out_;
    }

    // --- emergency stop: B, always, in every state
    if (rising(in.b, prev_.b)) emergencyStop();

    // --- reset of the E-stop: Menu held, E-stop active, disarmed
    if (out_.estop && !out_.armed && in.menu) {
        resetHeld_ += dt;
        if (resetHeld_ >= cfg_.resetHoldS) {
            out_.estop = false;
            resetHeld_ = 0;
            aReleasedSinceArm_ = false;
        }
    } else {
        resetHeld_ = 0;
    }
    out_.resetProgress = out_.estop ? clampf(resetHeld_ / cfg_.resetHoldS, 0, 1) : 0;

    // --- arm: A held for 2 s while disarmed; a press while armed disarms
    if (!in.a) aReleasedSinceArm_ = true;
    if (out_.armed) {
        if (rising(in.a, prev_.a) && aReleasedSinceArm_) disarm();
        out_.armProgress = 0;
    } else if (in.a && aReleasedSinceArm_ && !out_.estop) {
        armHeld_ += dt;
        if (armHeld_ >= cfg_.armHoldS) {
            out_.armed = true;
            armHeld_ = 0;
            aReleasedSinceArm_ = false;                // the same hold must not disarm again
        }
        out_.armProgress = out_.armed ? 0 : clampf(armHeld_ / cfg_.armHoldS, 0, 1);
    } else {
        armHeld_ = 0;
        out_.armProgress = 0;
    }

    // --- page
    out_.nextPage = rising(in.view, prev_.view);

    // --- walking, yaw, height rate: only while armed (nothing waits in the sticks when arming)
    if (out_.armed) {
        float x = shape(in.leftX, 0, 0), y = shape(in.leftY, 0, 0);
        float mag = std::sqrt(x * x + y * y);                          // radial dead zone, direction kept
        float scaled = shape(mag, cfg_.deadZone, cfg_.expo);
        const float k = mag > 1e-6f ? scaled / mag : 0.0f;
        const float speed = cfg_.speedMin + (cfg_.speedMax - cfg_.speedMin) * clampf(in.r2, 0, 1);
        float vx = x * k * speed, vy = y * k * speed;
        const float vm = std::sqrt(vx * vx + vy * vy);
        if (vm > 1.0f) { vx /= vm; vy /= vm; }
        out_.strafe = vx;
        out_.forward = vy;
        out_.turn = shape(in.rightX, cfg_.deadZone, cfg_.expo);
        height01_ = clampf(height01_ + shape(in.rightY, cfg_.deadZone, cfg_.expo) * cfg_.heightRate01 * dt, 0.0f, 1.0f);
    } else {
        out_.strafe = out_.forward = out_.turn = 0;
    }
    out_.height = 2.0f * height01_ - 1.0f;

    // --- gait, solver, step height (buttons work while armed or not)
    if (rising(in.x, prev_.x)) ripple_ = !ripple_;
    if (rising(in.dpadLeft, prev_.dpadLeft)) solver_ = std::max(0, solver_ - 1);
    if (rising(in.dpadRight, prev_.dpadRight)) solver_ = std::min(2, solver_ + 1);
    if (rising(in.dpadUp, prev_.dpadUp)) step01_ = clampf(step01_ + cfg_.stepStep01, 0, 1);
    if (rising(in.dpadDown, prev_.dpadDown)) step01_ = clampf(step01_ - cfg_.stepStep01, 0, 1);
    out_.gaitRipple = ripple_;
    out_.solver = solver_;
    out_.stepHeight = 2.0f * step01_ - 1.0f;

    // --- right pad: tilt relative to where the finger landed; lifting returns to the trim, or records with R4
    const Touch& rp = in.rightPad;
    if (rp.down && !rightTouching_) {
        rightTouching_ = true;
        rightStartX_ = rp.x;
        rightStartY_ = rp.y;
    }
    if (rightTouching_) {
        if (rp.down) {
            rollLive_ = clampf((rp.x - rightStartX_) * cfg_.tiltGain, -1, 1);       // right = right side down
            pitchLive_ = clampf((rp.y - rightStartY_) * cfg_.tiltGain, -1, 1);      // up = nose down
        } else {
            rightTouching_ = false;
            if (in.r4 || prev_.r4) {                                               // record: the tilt stays
                rollTrim_ = clampf(rollTrim_ + rollLive_, -1, 1);
                pitchTrim_ = clampf(pitchTrim_ + pitchLive_, -1, 1);
            }
            rollLive_ = pitchLive_ = 0;
        }
    }
    if (rising(in.rightPadClick, prev_.rightPadClick)) rollTrim_ = pitchTrim_ = 0;
    out_.roll = clampf(rollTrim_ + rollLive_, -1, 1);
    out_.pitch = clampf(pitchTrim_ + pitchLive_, -1, 1);
    out_.tiltRecorded = rollTrim_ != 0 || pitchTrim_ != 0;

    // --- left pad: stance (Y only), same behaviour
    const Touch& lp = in.leftPad;
    if (lp.down && !leftTouching_) {
        leftTouching_ = true;
        leftStartY_ = lp.y;
    }
    if (leftTouching_) {
        if (lp.down) {
            stanceLive_ = clampf((lp.y - leftStartY_) * cfg_.stanceGain, -1, 1);    // up = wider
        } else {
            leftTouching_ = false;
            if (in.l4 || prev_.l4) stanceTrim_ = clampf(stanceTrim_ + stanceLive_, -1, 1);
            stanceLive_ = 0;
        }
    }
    if (rising(in.leftPadClick, prev_.leftPadClick)) stanceTrim_ = 0;
    out_.stance = clampf(stanceTrim_ + stanceLive_, -1, 1);
    out_.stanceRecorded = stanceTrim_ != 0;

    prev_ = in;
    return out_;
}

}  // namespace deck

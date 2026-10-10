#include "ChannelMapper.h"

#include <cmath>

namespace deck {

uint16_t valueToUs(float v) {
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (uint16_t)std::lround(kUsMid + v * (kUsMax - kUsMid));
}

Channels mapChannels(const ControlOutput& out, const ChannelMap& map) {
    Channels c;
    c.us.fill(kUsMid);
    // switches that are not mapped must still be "off", not centred
    auto put = [&](Function f, float v) {
        const int ch = map.channel[f];
        if (ch < 1 || ch > kChannels) return;
        float val = v;
        if (f <= FnTurn || f == FnRoll || f == FnPitch) {
            static const int bit[] = {0, 1, 2, 3, -1, 4, 5};
            if (map.invertMask & (1u << bit[f])) val = -val;
        }
        c.us[ch - 1] = valueToUs(val);
    };
    put(FnStrafe, out.strafe);
    put(FnForward, out.forward);
    put(FnHeight, out.height);
    put(FnTurn, out.turn);
    put(FnArm, out.armed && !out.estop ? 1.0f : -1.0f);
    put(FnRoll, out.roll);
    put(FnPitch, out.pitch);
    put(FnGait, out.gaitRipple ? 1.0f : -1.0f);
    put(FnSolver, out.solver == 0 ? -1.0f : (out.solver == 2 ? 1.0f : 0.0f));
    put(FnStepHeight, out.stepHeight);
    put(FnStance, out.stance);
    put(FnEstop, out.estop ? 1.0f : -1.0f);
    return c;
}

}  // namespace deck

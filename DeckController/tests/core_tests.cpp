// Self-tests of the control model and the channel mapper (no hardware, no Qt, no SDL).
#include <cmath>
#include <cstdio>

#include "core/ChannelMapper.h"
#include "core/ControlModel.h"

using namespace deck;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
#define NEAR(a, b, e) do { double _a = (a), _b = (b); if (std::fabs(_a - _b) > (e)) { std::printf("FAIL %s:%d  %s = %g, expected %g\n", __FILE__, __LINE__, #a, _a, _b); ++g_fail; } } while (0)

static InputState pad() { InputState s; s.connected = true; return s; }

static void run(ControlModel& m, const InputState& s, float seconds, float dt = 0.004f) {
    for (float t = 0; t < seconds; t += dt) m.update(s, dt);
}

static void armIt(ControlModel& m) {
    InputState s = pad();
    s.a = true;
    run(m, s, 2.05f);
    s.a = false;
    run(m, s, 0.1f);
}

static void testArming() {
    ControlModel m;
    InputState s = pad();
    CHECK(!m.output().armed);
    s.a = true;
    run(m, s, 1.0f);
    CHECK(!m.output().armed);
    NEAR(m.output().armProgress, 0.5, 0.05);
    s.a = false;                      // released early: nothing happens
    run(m, s, 0.1f);
    CHECK(!m.output().armed);
    NEAR(m.output().armProgress, 0, 1e-6);
    s.a = true;
    run(m, s, 2.05f);
    CHECK(m.output().armed);
    run(m, s, 1.0f);                  // the same hold does not disarm again
    CHECK(m.output().armed);
    s.a = false;
    run(m, s, 0.1f);
    s.a = true;                       // a press disarms
    run(m, s, 0.05f);
    CHECK(!m.output().armed);
}

static void testSticksOnlyWhenArmed() {
    ControlModel m;
    InputState s = pad();
    s.leftY = 1; s.rightX = 1; s.rightY = 1;
    run(m, s, 0.5f);
    NEAR(m.output().forward, 0, 1e-6);
    NEAR(m.output().turn, 0, 1e-6);
    NEAR(m.output().height, 0, 1e-6);          // height held, not moved
    armIt(m);
    NEAR(m.output().forward, 0, 1e-6);         // sticks centred when arming
    s.leftY = 1;
    s.r2 = 0;
    run(m, s, 0.1f);
    NEAR(m.output().forward, 0.4, 0.01);       // R2 released: 40 %
    s.r2 = 1;
    run(m, s, 0.1f);
    NEAR(m.output().forward, 1.0, 0.01);
    s.leftX = 1;                               // diagonal stays inside the unit circle
    run(m, s, 0.1f);
    CHECK(std::hypot(m.output().strafe, m.output().forward) <= 1.001f);
    s.leftX = 0.05f; s.leftY = 0.05f;          // inside the dead zone
    run(m, s, 0.1f);
    NEAR(m.output().strafe, 0, 1e-6);
    NEAR(m.output().forward, 0, 1e-6);
}

static void testHeightRate() {
    ControlModel m;
    armIt(m);
    InputState s = pad();
    s.rightY = 1;
    run(m, s, 1.0f);                           // 0.5 of the range per second -> +1.0 on the channel scale
    NEAR(m.output().height, 1.0, 0.02);
    s.rightY = 0;
    run(m, s, 1.0f);
    NEAR(m.output().height, 1.0, 0.02);        // stays
    s.rightY = -0.5f;
    run(m, s, 0.5f);
    CHECK(m.output().height < 1.0f);
    CHECK(m.output().height > -1.0f);
}

static void testEstopAndReset() {
    ControlModel m;
    armIt(m);
    CHECK(m.output().armed);
    InputState s = pad();
    s.b = true;
    run(m, s, 0.02f);
    CHECK(m.output().estop);
    CHECK(!m.output().armed);
    s.b = false;
    s.a = true;                                // cannot arm during an E-stop
    run(m, s, 2.5f);
    CHECK(!m.output().armed);
    s.a = false;
    s.menu = true;                             // reset needs 2 s
    run(m, s, 1.0f);
    CHECK(m.output().estop);
    NEAR(m.output().resetProgress, 0.5, 0.05);
    s.menu = false;
    run(m, s, 0.1f);
    CHECK(m.output().estop);
    s.menu = true;
    run(m, s, 2.05f);
    CHECK(!m.output().estop);
    CHECK(!m.output().armed);                  // stays disarmed
    // B also works while disarmed
    s = pad();
    s.b = true;
    run(m, s, 0.02f);
    CHECK(m.output().estop);
}

static void testControllerLossDisarms() {
    ControlModel m;
    armIt(m);
    CHECK(m.output().armed);
    InputState gone;                           // connected = false
    m.update(gone, 0.004f);
    CHECK(!m.output().armed);
    CHECK(m.output().controllerLost);
    CHECK(!m.output().estop);
}

static void testTiltAndRecord() {
    ControlModel m;
    InputState s = pad();
    s.rightPad = {true, 0.5f, 0.5f};
    run(m, s, 0.05f);
    s.rightPad = {true, 0.6f, 0.7f};           // moved right 0.1 and up 0.2
    run(m, s, 0.05f);
    NEAR(m.output().roll, 0.2, 0.01);
    NEAR(m.output().pitch, 0.4, 0.01);
    s.rightPad.down = false;                   // lift: back to straight
    run(m, s, 0.05f);
    NEAR(m.output().roll, 0, 1e-6);
    NEAR(m.output().pitch, 0, 1e-6);
    // record with R4
    s.rightPad = {true, 0.5f, 0.5f};
    run(m, s, 0.05f);
    s.rightPad = {true, 0.6f, 0.5f};
    s.r4 = true;
    run(m, s, 0.05f);
    s.rightPad.down = false;
    run(m, s, 0.05f);
    s.r4 = false;
    run(m, s, 0.05f);
    NEAR(m.output().roll, 0.2, 0.01);
    CHECK(m.output().tiltRecorded);
    // a second touch adds to the trim
    s.rightPad = {true, 0.5f, 0.5f};
    run(m, s, 0.05f);
    s.rightPad = {true, 0.55f, 0.5f};
    run(m, s, 0.05f);
    NEAR(m.output().roll, 0.3, 0.01);
    s.rightPad.down = false;
    run(m, s, 0.05f);
    NEAR(m.output().roll, 0.2, 0.01);
    // click resets
    s.rightPadClick = true;
    run(m, s, 0.02f);
    NEAR(m.output().roll, 0, 1e-6);
    CHECK(!m.output().tiltRecorded);
}

static void testStance() {
    ControlModel m;
    InputState s = pad();
    s.leftPad = {true, 0.3f, 0.5f};
    run(m, s, 0.05f);
    s.leftPad = {true, 0.9f, 0.6f};            // X is ignored, Y up 0.1
    run(m, s, 0.05f);
    NEAR(m.output().stance, 0.2, 0.01);
    s.l4 = true;
    s.leftPad.down = false;
    run(m, s, 0.05f);
    s.l4 = false;
    run(m, s, 0.05f);
    NEAR(m.output().stance, 0.2, 0.01);
    s.leftPadClick = true;
    run(m, s, 0.02f);
    NEAR(m.output().stance, 0, 1e-6);
}

static void testButtons() {
    ControlModel m;
    InputState s = pad();
    s.x = true; run(m, s, 0.02f); s.x = false; run(m, s, 0.02f);
    CHECK(m.output().gaitRipple);
    s.x = true; run(m, s, 0.02f); s.x = false; run(m, s, 0.02f);
    CHECK(!m.output().gaitRipple);
    s.dpadRight = true; run(m, s, 0.02f); s.dpadRight = false; run(m, s, 0.02f);
    CHECK(m.output().solver == 2);
    s.dpadRight = true; run(m, s, 0.02f); s.dpadRight = false; run(m, s, 0.02f);
    CHECK(m.output().solver == 2);             // limited
    s.dpadLeft = true; run(m, s, 0.02f); s.dpadLeft = false; run(m, s, 0.02f);
    CHECK(m.output().solver == 1);
    const float before = m.output().stepHeight;
    s.dpadUp = true; run(m, s, 0.02f); s.dpadUp = false; run(m, s, 0.02f);
    NEAR(m.output().stepHeight, before + 0.25, 1e-4);   // 5 mm of 40 mm = 0.125 of the range = 0.25 on -1..1
    s.view = true; m.update(s, 0.004f);
    CHECK(m.output().nextPage);
    m.update(s, 0.004f);
    CHECK(!m.output().nextPage);
}

static void testChannelMapper() {
    ChannelMap map;
    ControlOutput o;
    o.strafe = 1; o.forward = -1; o.height = 0; o.turn = 0.5f; o.roll = -1; o.pitch = 1;
    o.armed = true; o.gaitRipple = true; o.solver = 0; o.stepHeight = 0.25f; o.stance = -0.5f; o.estop = false;
    Channels c = mapChannels(o, map);
    CHECK(c.us[0] == 2012);                    // CH1 strafe
    CHECK(c.us[1] == 988);                     // CH2 forward
    CHECK(c.us[2] == 1500);                    // CH3 height
    CHECK(c.us[3] == 1756);                    // CH4 turn
    CHECK(c.us[4] == 2012);                    // CH5 arm
    CHECK(c.us[5] == 988);                     // CH6 roll
    CHECK(c.us[6] == 2012);                    // CH7 pitch
    CHECK(c.us[7] == 2012);                    // CH8 gait ripple
    CHECK(c.us[8] == 988);                     // CH9 analytic
    CHECK(c.us[11] == 988);                    // CH12 E-stop off
    o.estop = true;
    c = mapChannels(o, map);
    CHECK(c.us[11] == 2012);
    CHECK(c.us[4] == 988);                     // an E-stop always shows as disarmed
    // remapped + inverted
    map.channel[FnForward] = 13;
    map.invertMask = 1u << 1;
    o.estop = false; o.forward = 1;
    c = mapChannels(o, map);
    CHECK(c.us[12] == 988);
    CHECK(c.us[1] == 1500);                    // CH2 unused now
    map.channel[FnGait] = 0;                   // not used
    c = mapChannels(o, map);
    CHECK(c.us[7] == 1500);
}

int main() {
    testArming();
    testSticksOnlyWhenArmed();
    testHeightRate();
    testEstopAndReset();
    testControllerLossDisarms();
    testTiltAndRecord();
    testStance();
    testButtons();
    testChannelMapper();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("all core tests passed\n");
    return 0;
}

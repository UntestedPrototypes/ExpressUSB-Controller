// The app against the robot's own code (PentaPod firmware sources, read only): what the app sends comes out of the robot's RcMapper as the
// intended motion, and what the robot's telemetry builder produces is understood by the TelemetryParser.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "control/rc_map.h"
#include "control/rc_mapper.h"
#include "drivers/crsf_telemetry.h"
#include "core/ChannelMapper.h"
#include "core/ControlModel.h"
#include "core/TelemetryParser.h"

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
#define NEAR(a, b, e) do { double _a = (a), _b = (b); if (std::fabs(_a - _b) > (e)) { std::printf("FAIL %s:%d  %s = %g, expected %g\n", __FILE__, __LINE__, #a, _a, _b); ++g_fail; } } while (0)

// The library's microseconds -> CRSF value (988 us = 172, 2012 us = 1811)
static uint16_t crsfValue(uint16_t us) { return uint16_t(172 + (int(us) - 988) * (1811 - 172) / (2012 - 988)); }

struct Robot {
    pentapod::RcMapping map = pentapod::rcLayoutFullRes();
    pentapod::GaitLimits lim;
    pentapod::RcMapper mapper{map, lim};
    pentapod::MotionCommand cmd;
    pentapod::RadioStatus st;
    void feed(const deck::Channels& c) {
        uint16_t ch[16];
        for (int i = 0; i < 16; ++i) ch[i] = crsfValue(c.us[size_t(i)]);
        mapper.update(ch, true, cmd, st);
    }
};

static void testMappingThroughTheRobot() {
    Robot r;
    deck::ChannelMap map;
    deck::ControlOutput o;
    r.feed(deck::mapChannels(o, map));            // disarmed, link up: allows arming later
    CHECK(!r.st.armed);
    o.armed = true;
    r.feed(deck::mapChannels(o, map));
    CHECK(r.st.armed);
    CHECK(!r.st.estop);

    o.forward = 1;                                // full forward
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.speed_mps, r.lim.speed_max, 0.02 * r.lim.speed_max);
    NEAR(r.cmd.gait.direction_rad, 0, 0.05);
    o.forward = 0;
    o.strafe = 1;                                 // full right: the robot's +y is left, so the direction is -90 degrees
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.direction_rad, -1.5708, 0.05);
    o.strafe = 0;
    o.turn = 1;                                   // clockwise = negative yaw rate
    r.feed(deck::mapChannels(o, map));
    CHECK(r.cmd.gait.yaw_rate_rps < -0.1f);
    o.turn = 0;
    o.height = -1;
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.body_height_m, r.map.body_height_min, 0.002);
    o.height = 1;
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.body_height_m, r.map.body_height_max, 0.002);
    o.roll = 0.5f; o.pitch = -0.5f;
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.roll_rad, 0.5 * r.lim.tilt_max, 0.01);
    NEAR(r.cmd.gait.pitch_rad, -0.5 * r.lim.tilt_max, 0.01);
    o.gaitRipple = true; o.solver = 2;
    r.feed(deck::mapChannels(o, map));
    CHECK(r.cmd.gait.gait == pentapod::GaitType::Ripple);
    CHECK(r.cmd.hasSolver && r.cmd.solver == pentapod::IkSolverType::Numeric);
    o.solver = 0;
    r.feed(deck::mapChannels(o, map));
    CHECK(r.cmd.solver == pentapod::IkSolverType::Analytic);
    o.stepHeight = -1;
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.step_height_m, r.map.step_height_min, 0.001);
    o.stance = 1;
    r.feed(deck::mapChannels(o, map));
    NEAR(r.cmd.gait.stance_m, r.map.stance_max, 0.002);

    // B: the E-stop channel releases the motors on the robot, and the radio is disarmed
    o.estop = true; o.armed = false;
    r.feed(deck::mapChannels(o, map));
    CHECK(r.st.estop && r.st.estopSet);
    CHECK(!r.st.armed);
    // Menu: channel off with the arm channel off = reset
    o.estop = false;
    r.feed(deck::mapChannels(o, map));
    CHECK(r.st.estopReset);
}

static void testTelemetryFromTheRobot() {
    pentapod::RobotStatus rs;
    rs.systemState = 1; rs.controllerPhase = 2; rs.posture = 'I';
    rs.maxTempServo = 8; rs.maxTempC = 61; rs.batteryMv = 11640; rs.supplyDv = 74;
    rs.solver = 0; rs.bodyHeightMm = 140; rs.stepHeightMm = 35; rs.stanceMm = 200; rs.speedMmS = 45;
    rs.rollRad = 0.1f; rs.pitchRad = -0.2f;
    deck::TelemetryParser p;
    pentapod::CrsfTelemetry tel;
    uint8_t f[80];
    for (uint32_t t = 0; t < 6000; t += 2) {
        const size_t n = tel.next(t, rs, f, sizeof(f));
        if (n) p.onFrame(f[2], f + 3, f[1] - 2, t);   // type, payload without type and crc
    }
    const deck::RobotState& s = p.state();
    CHECK(s.haveStatus && s.state == 'A' && s.phase == 2 && s.posture == 'I');
    CHECK(s.fault == 0 && s.legMask == 0);
    CHECK(s.haveDetail);
    CHECK(s.hotServo == 8 && s.hotTempC == 61);
    NEAR(s.packV, 11.6, 0.01);
    NEAR(s.supplyV, 7.4, 0.01);
    CHECK(!s.ripple && s.solver == 'A');
    CHECK(s.bodyHeightMm == 140 && s.stepHeightMm == 35 && s.stanceMm == 200 && s.speedMmS == 45);
    CHECK(!s.seqRunning);
    CHECK(s.haveAttitude);
    NEAR(s.rollRad, 0.1, 0.0002);
    NEAR(s.pitchRad, -0.2, 0.0002);

    // a fault and a running sequence
    rs.systemState = 3; rs.faultCode = 3; rs.posture = 'X'; rs.faultServo = 8; rs.disabledLegMask = 0x04;
    bool changed = false;
    for (uint32_t t = 6000; t < 9000; t += 2) {
        const size_t n = tel.next(t, rs, f, sizeof(f));
        if (n) changed = p.onFrame(f[2], f + 3, f[1] - 2, t) || changed;
    }
    CHECK(changed);
    CHECK(s.posture == 'X' && s.fault == 3 && s.faultServo == 8 && s.legMask == 4);
    CHECK(!s.text.empty());
    rs.systemState = 1; rs.faultCode = 0; rs.posture = 'Q'; rs.seqRunning = true; rs.seqStep = 2; rs.seqCount = 3;
    std::strcpy(rs.seqName, "fold");
    for (uint32_t t = 9000; t < 14000; t += 2) {
        const size_t n = tel.next(t, rs, f, sizeof(f));
        if (n) p.onFrame(f[2], f + 3, f[1] - 2, t);
    }
    CHECK(s.posture == 'Q' && s.seqRunning && s.seqName == "fold" && s.seqStep == 2 && s.seqCount == 3);
}

int main() {
    testMappingThroughTheRobot();
    testTelemetryFromTheRobot();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("all interop tests passed\n");
    return 0;
}

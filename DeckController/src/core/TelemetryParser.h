#pragma once
// Decodes what the PentaPod says over the CRSF downlink (frame types the ExpressUSB library hands out through onCrsfFrame):
//   0x21 flight mode text: the status line, or the detail line (starts with '#')   0x1E attitude   (battery comes from the library)
// Formats: see PentaPod src/drivers/crsf_telemetry.h. Pure C++.
#include <cstddef>
#include <cstdint>
#include <string>

namespace deck {

struct RobotState {
    // status line:  <S><P><Q> F<f> W<w> S<ss> L<mm> <text>
    bool haveStatus = false;
    char state = '?';          // W waiting, A active, D active direct, X failsafe
    int phase = 0;             // controller phase 0..4
    char posture = '?';        // F folded, B standby, U standing up, I idle, L lowering, Q sequence, X failsafe, ? unknown
    int fault = 0, stopReason = 0, faultServo = 0, legMask = 0;
    std::string text;
    uint32_t statusAtMs = 0;
    // detail line
    bool haveDetail = false;
    int hotServo = 0, hotTempC = 0;
    float packV = 0, supplyV = 0;
    bool ripple = false;
    char solver = 'A';         // A analytic, M mixed (auto), N numeric
    int bodyHeightMm = 0, stepHeightMm = 0, stanceMm = 0, speedMmS = 0;
    bool seqRunning = false;
    std::string seqName;
    int seqStep = 0, seqCount = 0;
    uint32_t detailAtMs = 0;
    // attitude frame: the tilt the robot applies
    bool haveAttitude = false;
    float rollRad = 0, pitchRad = 0, yawRateRps = 0;
    uint32_t attitudeAtMs = 0;
};

const char* postureName(char posture);
const char* stateName(char state);
const char* faultName(int code);   // the latched fault / stop reason codes

class TelemetryParser {
public:
    // One CRSF frame (type, payload without type and CRC). Returns true when the status line changed (posture, state, fault ...).
    bool onFrame(uint8_t type, const uint8_t* payload, size_t length, uint32_t nowMs);
    bool parseStatus(const std::string& text, uint32_t nowMs);
    bool parseDetail(const std::string& text, uint32_t nowMs);
    const RobotState& state() const { return s_; }
    void reset() { s_ = RobotState(); }

private:
    RobotState s_;
};

}  // namespace deck

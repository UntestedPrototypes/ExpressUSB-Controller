#include "TelemetryParser.h"

#include <cstdio>
#include <cstring>

namespace deck {

const char* postureName(char p) {
    switch (p) {
    case 'F': return "Folded";
    case 'B': return "Standby";
    case 'U': return "Standing up";
    case 'I': return "Idle";
    case 'L': return "Lowering";
    case 'Q': return "Sequence";
    case 'X': return "FAILSAFE";
    default: return "Unknown";
    }
}

const char* stateName(char s) {
    switch (s) {
    case 'W': return "Motors off";
    case 'A': return "Active";
    case 'D': return "Direct control";
    case 'X': return "Failsafe";
    default: return "?";
    }
}

const char* faultName(int code) {
    switch (code) {
    case 0: return "none";
    case 1: return "E-stop";
    case 2: return "command timeout";
    case 3: return "joint limit";
    case 4: return "crash";
    case 5: return "servo comms";
    case 6: return "CLI";
    case 7: return "control stall";
    case 8: return "over temperature";
    case 9: return "under voltage";
    default: return "fault";
    }
}

bool TelemetryParser::onFrame(uint8_t type, const uint8_t* p, size_t n, uint32_t nowMs) {
    if (type == 0x21 && n > 0) {                       // flight mode text, zero terminated
        size_t len = 0;
        while (len < n && p[len]) ++len;
        const std::string text(reinterpret_cast<const char*>(p), len);
        if (!text.empty() && text[0] == '#') {
            parseDetail(text, nowMs);
            return false;
        }
        return parseStatus(text, nowMs);
    }
    if (type == 0x1E && n >= 6) {                      // attitude: pitch, roll, yaw; int16 big endian, 0.0001 rad
        auto rd = [&](size_t i) { return float(int16_t((p[i] << 8) | p[i + 1])) / 10000.0f; };
        s_.pitchRad = rd(0);
        s_.rollRad = rd(2);
        s_.yawRateRps = rd(4);
        s_.haveAttitude = true;
        s_.attitudeAtMs = nowMs;
    }
    return false;
}

bool TelemetryParser::parseStatus(const std::string& t, uint32_t nowMs) {
    char st = 0, po = 0;
    unsigned ph = 0, f = 0, w = 0, ss = 0, lm = 0;
    int used = 0;
    // "A2I F0 W0 S00 L00 text"
    if (std::sscanf(t.c_str(), "%c%1u%c F%u W%u S%u L%x %n", &st, &ph, &po, &f, &w, &ss, &lm, &used) < 7) return false;
    RobotState n = s_;
    n.haveStatus = true;
    n.state = st;
    n.phase = int(ph);
    n.posture = po;
    n.fault = int(f);
    n.stopReason = int(w);
    n.faultServo = int(ss);
    n.legMask = int(lm);
    n.text = used > 0 ? t.substr(size_t(used)) : std::string();
    n.statusAtMs = nowMs;
    const bool changed = !s_.haveStatus || n.state != s_.state || n.posture != s_.posture || n.fault != s_.fault ||
                         n.stopReason != s_.stopReason || n.legMask != s_.legMask || n.text != s_.text;
    s_ = n;
    return changed;
}

bool TelemetryParser::parseDetail(const std::string& t, uint32_t nowMs) {
    unsigned servo = 0, temp = 0, h = 0, sh = 0, stn = 0, v = 0;
    float pack = 0, sup = 0;
    char gait = 0, solver = 0;
    int used = 0;
    // "#T08:61 B11.6 U7.4 GW QA H140 SH035 ST200 V045 [SEQfold:2/3]"
    if (std::sscanf(t.c_str(), "#T%u:%u B%f U%f G%c Q%c H%u SH%u ST%u V%u%n", &servo, &temp, &pack, &sup, &gait, &solver, &h, &sh, &stn, &v,
                    &used) < 10)
        return false;
    s_.haveDetail = true;
    s_.hotServo = int(servo);
    s_.hotTempC = int(temp);
    s_.packV = pack;
    s_.supplyV = sup;
    s_.ripple = gait == 'R';
    s_.solver = solver;
    s_.bodyHeightMm = int(h);
    s_.stepHeightMm = int(sh);
    s_.stanceMm = int(stn);
    s_.speedMmS = int(v);
    s_.seqRunning = false;
    s_.seqName.clear();
    const char* rest = t.c_str() + used;
    if (std::strncmp(rest, " SEQ", 4) == 0) {
        const char* name = rest + 4;
        const char* colon = std::strchr(name, ':');
        if (colon) {
            s_.seqRunning = true;
            s_.seqName.assign(name, size_t(colon - name));
            unsigned step = 0, count = 0;
            if (std::sscanf(colon, ":%u/%u", &step, &count) >= 1) {
                s_.seqStep = int(step);
                s_.seqCount = int(count);
            }
        }
    }
    s_.detailAtMs = nowMs;
    return true;
}

}  // namespace deck

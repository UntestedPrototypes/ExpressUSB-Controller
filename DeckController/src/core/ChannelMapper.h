#pragma once
// Control values -> RC channels in microseconds (988..2012 us = the full CRSF range), following the robot's channel map
// (RcMap, default = the full resolution layout with the E-stop on CH12).
#include <array>
#include <cstdint>

#include "ControlModel.h"

namespace deck {

enum Function { FnStrafe, FnForward, FnHeight, FnTurn, FnArm, FnRoll, FnPitch, FnGait, FnSolver, FnStepHeight, FnStance, FnEstop, kFunctions };

struct ChannelMap {
    // channel 1..16, 0 = not used
    std::array<uint8_t, kFunctions> channel{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    // invert bits, same order as the robot's RcMap.invert_mask for the analog ones: 0 strafe 1 forward 2 height 3 turn 4 roll 5 pitch
    uint8_t invertMask = 0;
};

constexpr int kChannels = 16;
constexpr uint16_t kUsMin = 988, kUsMid = 1500, kUsMax = 2012;

struct Channels {
    std::array<uint16_t, kChannels> us{};
};

// Every channel not in the map stays at the centre (the arm and E-stop channels at their "off" end are handled by the map)
Channels mapChannels(const ControlOutput& out, const ChannelMap& map);

uint16_t valueToUs(float v);   // -1..+1 -> 988..2012 us

}  // namespace deck

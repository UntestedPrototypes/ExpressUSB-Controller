#pragma once
// What the controller reports, already in the app's conventions (independent of SDL, so it can be tested and simulated):
//   sticks:  -1..+1, x right = +, y UP = +
//   touch:   0..1 across the pad, x right = +, y UP = +
//   trigger: 0..1
namespace deck {

struct Touch {
    bool down = false;
    float x = 0.5f, y = 0.5f;
};

struct InputState {
    bool connected = false;                 // a controller is present
    float leftX = 0, leftY = 0;             // walk: y up = forward, x right = right
    float rightX = 0, rightY = 0;           // x = yaw (right = clockwise), y = body height rate (up = rise)
    float r2 = 0;                           // speed scale
    Touch leftPad, rightPad;
    bool leftPadClick = false, rightPadClick = false;
    bool a = false, b = false, x = false;
    bool menu = false, view = false;
    bool dpadUp = false, dpadDown = false, dpadLeft = false, dpadRight = false;
    bool l4 = false, r4 = false;            // back grips: record stance / tilt
};

}  // namespace deck

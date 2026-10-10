#include "InputReader.h"

#include <SDL3/SDL.h>

#include <cmath>

namespace {
float axis(SDL_Gamepad* p, SDL_GamepadAxis a) {
    const Sint16 v = SDL_GetGamepadAxis(p, a);
    const float f = v < 0 ? v / 32768.0f : v / 32767.0f;
    return f < -1 ? -1 : (f > 1 ? 1 : f);
}
bool btn(SDL_Gamepad* p, SDL_GamepadButton b) { return SDL_GetGamepadButton(p, b); }
}  // namespace

InputReader::InputReader() {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");   // the app keeps reading; losing focus is handled by the app itself
    m_ok = SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_EVENTS);
}

InputReader::~InputReader() {
    close();
    if (m_ok) SDL_Quit();
}

void InputReader::open() {
    int n = 0;
    SDL_JoystickID* ids = SDL_GetGamepads(&n);
    if (ids && n > 0) {
        m_pad = SDL_OpenGamepad(ids[0]);
        if (m_pad) {
            const char* name = SDL_GetGamepadName(m_pad);
            m_name = name ? QString::fromUtf8(name) : QStringLiteral("Controller");
            m_touchpads = SDL_GetNumGamepadTouchpads(m_pad);
        }
    }
    SDL_free(ids);
}

void InputReader::close() {
    if (m_pad) SDL_CloseGamepad(m_pad);
    m_pad = nullptr;
    m_name.clear();
    m_touchpads = 0;
}

deck::InputState InputReader::poll() {
    deck::InputState s;
    if (!m_ok) return s;
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_EVENT_GAMEPAD_REMOVED && m_pad && e.gdevice.which == SDL_GetGamepadID(m_pad)) close();
    }
    if (!m_pad) open();
    if (!m_pad || !SDL_GamepadConnected(m_pad)) {
        close();
        return s;
    }

    s.connected = true;
    s.leftX = axis(m_pad, SDL_GAMEPAD_AXIS_LEFTX);
    s.leftY = -axis(m_pad, SDL_GAMEPAD_AXIS_LEFTY);        // SDL: down = +
    s.rightX = axis(m_pad, SDL_GAMEPAD_AXIS_RIGHTX);
    s.rightY = -axis(m_pad, SDL_GAMEPAD_AXIS_RIGHTY);
    const float r2 = axis(m_pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER);
    s.r2 = r2 < 0 ? 0 : r2;
    s.a = btn(m_pad, SDL_GAMEPAD_BUTTON_SOUTH);
    s.b = btn(m_pad, SDL_GAMEPAD_BUTTON_EAST);
    s.x = btn(m_pad, SDL_GAMEPAD_BUTTON_WEST);
    s.menu = btn(m_pad, SDL_GAMEPAD_BUTTON_START);
    s.view = btn(m_pad, SDL_GAMEPAD_BUTTON_BACK);
    s.dpadUp = btn(m_pad, SDL_GAMEPAD_BUTTON_DPAD_UP);
    s.dpadDown = btn(m_pad, SDL_GAMEPAD_BUTTON_DPAD_DOWN);
    s.dpadLeft = btn(m_pad, SDL_GAMEPAD_BUTTON_DPAD_LEFT);
    s.dpadRight = btn(m_pad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT);
    s.l4 = btn(m_pad, SDL_GAMEPAD_BUTTON_LEFT_PADDLE1);
    s.r4 = btn(m_pad, SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1);

    // Touchpads. SDL numbers them in the controller's order; on the Deck 0 should be the left and 1 the right pad (to be verified on the
    // device, milestone 1). A controller with a single pad (DualShock / DualSense) is treated as the right (tilt) pad.
    // Touch y is 0 at the top: flipped here so up = +.
    auto readPad = [&](int index, deck::Touch& t) {
        bool down = false;
        float x = 0, y = 0, pressure = 0;
        if (SDL_GetGamepadTouchpadFinger(m_pad, index, 0, &down, &x, &y, &pressure)) {
            t.down = down;
            t.x = x;
            t.y = 1.0f - y;
        }
    };
    if (m_touchpads >= 2) {
        readPad(0, s.leftPad);
        readPad(1, s.rightPad);
    } else if (m_touchpads == 1) {
        readPad(0, s.rightPad);
    }
    s.rightPadClick = btn(m_pad, SDL_GAMEPAD_BUTTON_TOUCHPAD);   // a pad click as its own button is not exposed for the Deck: to verify
    return s;
}

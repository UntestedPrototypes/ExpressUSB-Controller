#pragma once
// Reads the first game controller with SDL3 (sticks, triggers, buttons, back grips and, on the Steam Deck, the two touchpads) and
// reports it as an InputState.
#include <QString>

#include "core/InputState.h"

struct SDL_Gamepad;

class InputReader {
public:
    InputReader();
    ~InputReader();

    // Pumps SDL events (hot plug) and returns the current state; connected = false when no controller is present
    deck::InputState poll();

    QString controllerName() const { return m_name; }
    int touchpadCount() const { return m_touchpads; }

private:
    void open();
    void close();

    SDL_Gamepad* m_pad = nullptr;
    QString m_name;
    int m_touchpads = 0;
    bool m_ok = false;
};

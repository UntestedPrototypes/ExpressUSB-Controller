#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <cctype>

#ifdef _WIN32
#include <windows.h>
#include <xinput.h>
#include <conio.h>
int getKey() {
    if (_kbhit()) return _getch();
    return -1;
}
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
int getKey() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    return ch != EOF ? ch : -1;
}
#endif

// Cross-platform Gamepad Reader
bool getGamepad(int& roll, int& pitch, int& throttle, int& yaw, bool& armed) {
#ifdef _WIN32
    XINPUT_STATE state;
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    if (XInputGetState(0, &state) == ERROR_SUCCESS) {
        auto mapAxis = [](SHORT val) {
            if (std::abs(val) < 8000) return 1500;
            return 1500 + (val * 500 / 32767);
        };
        yaw = mapAxis(state.Gamepad.sThumbLX);
        throttle = mapAxis(state.Gamepad.sThumbLY); 
        roll = mapAxis(state.Gamepad.sThumbRX);
        pitch = mapAxis(state.Gamepad.sThumbRY); 
        
        if (state.Gamepad.wButtons & XINPUT_GAMEPAD_A) armed = true;
        if (state.Gamepad.wButtons & XINPUT_GAMEPAD_B) armed = false;
        return true;
    }
#else
    static int js_fd = -1;
    if (js_fd < 0) js_fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js_fd >= 0) {
        struct js_event jse;
        while (::read(js_fd, &jse, sizeof(jse)) > 0) {
            if (jse.type & JS_EVENT_AXIS) {
                int mapped = 1500 + (jse.value * 500 / 32767);
                if (jse.number == 0) yaw = mapped;
                if (jse.number == 1) throttle = 3000 - mapped;
                if (jse.number == 3) roll = mapped;
                if (jse.number == 4) pitch = 3000 - mapped;
            } else if (jse.type & JS_EVENT_BUTTON) {
                if (jse.number == 0 && jse.value == 1) armed = true;
                if (jse.number == 1 && jse.value == 1) armed = false;
            }
        }
        return true;
    }
#endif
    return false;
}

#include "mavlink_controller/mavlink_controller.h"

std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    g_running = false;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: rc_control_example <COM_PORT> [BAUD]" << std::endl;
        std::cerr << "Example: rc_control_example COM10 460800" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    uint32_t baud = (argc >= 3) ? std::stoul(argv[2]) : 460800;

    std::signal(SIGINT, signalHandler);

    mavlink_ctrl::MavlinkController controller;
    
    controller.onStateChange([](mavlink_ctrl::ConnectionState state) {
        switch (state) {
            case mavlink_ctrl::ConnectionState::Disconnected:
                std::cout << "\n[STATE] Disconnected" << std::endl;
                break;
            case mavlink_ctrl::ConnectionState::Connected:
                std::cout << "\n[STATE] Connected (waiting for remote heartbeat...)" << std::endl;
                break;
            case mavlink_ctrl::ConnectionState::LinkUp:
                std::cout << "\n[STATE] Link Up!" << std::endl;
                break;
        }
    });

    std::cout << "Connecting to " << port << " @ " << baud << " baud..." << std::endl;
    if (!controller.connect(port, baud)) {
        std::cerr << "Failed to open port " << port << std::endl;
        return 1;
    }

    std::cout << "Connected!\n\n";
    std::cout << "=== RC Controller (MAVLink) ===\n";
    std::cout << "Gamepad: Auto-detected (XInput / /dev/input/js0)\n";
    std::cout << "Keyboard (Fallback):\n";
    std::cout << "  W/S : Pitch\n";
    std::cout << "  A/D : Roll\n";
    std::cout << "  I/K : Throttle\n";
    std::cout << "  J/L : Yaw\n";
    std::cout << "  SPACE : Toggle Arm\n";
    std::cout << "  Q : Quit\n\n";

    int roll = 1500, pitch = 1500, throttle = 1500, yaw = 1500;
    bool armed = false;
    bool using_gamepad = false;

    while (g_running) {
        // 1. Try reading Gamepad First
        using_gamepad = getGamepad(roll, pitch, throttle, yaw, armed);
        
        // 2. If no gamepad, fallback to keyboard
        if (!using_gamepad) {
            int key = getKey();
            if (key != -1) {
                key = std::tolower(key);
                if (key == 'q') g_running = false;
                if (key == ' ') armed = !armed;
                
                if (key == 'w') pitch = 2000;
                else if (key == 's') pitch = 1000;
                else pitch = 1500;
                
                if (key == 'd') roll = 2000;
                else if (key == 'a') roll = 1000;
                else roll = 1500;
                
                if (key == 'i') throttle = 2000;
                else if (key == 'k') throttle = 1000;
                else throttle = 1500;
                
                if (key == 'l') yaw = 2000;
                else if (key == 'j') yaw = 1000;
                else yaw = 1500;
            } else {
                roll = 1500; pitch = 1500; throttle = 1500; yaw = 1500;
            }
        }

        // Apply to MAVLink RC override
        controller.setChannel(0, roll);
        controller.setChannel(1, pitch);
        controller.setChannel(2, throttle);
        controller.setChannel(3, yaw);
        controller.setChannel(4, armed ? 2000 : 1000);

        // Get radio link status
        auto t = controller.getTelemetry();

        // Print real-time values to the terminal
        std::cout << "\r[RC Out] [" << (using_gamepad ? "GAMEPAD" : "KEYBOARD") << "] "
                  << "R:" << roll 
                  << " P:" << pitch 
                  << " T:" << throttle 
                  << " Y:" << yaw 
                  << " ARM:" << (armed ? "ARMED   " : "DISARMED");
        
        if (t.radio_rssi > 0) {
            std::cout << " | RSSI:" << (int)t.radio_rssi;
        }
        
        std::cout << "    " << std::flush;

        // 50Hz update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "\nShutting down..." << std::endl;
    controller.disconnect();
    return 0;
}

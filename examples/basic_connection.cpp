#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <iomanip>

#include "mavlink_controller/mavlink_controller.h"

std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    g_running = false;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: expressusb_example <COM_PORT> [BAUD]" << std::endl;
        std::cerr << "Example: expressusb_example COM10 460800" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Available serial ports:" << std::endl;
        for (const auto& port : mavlink_ctrl::MavlinkController::listSerialPorts()) {
            std::cerr << "  " << port << std::endl;
        }
        return 1;
    }

    std::string port = argv[1];
    uint32_t baud = (argc >= 3) ? std::stoul(argv[2]) : 460800;

    std::signal(SIGINT, signalHandler);

    mavlink_ctrl::MavlinkController controller;

    // State change callback
    controller.onStateChange([](mavlink_ctrl::ConnectionState state) {
        switch (state) {
            case mavlink_ctrl::ConnectionState::Disconnected:
                std::cout << "[STATE] Disconnected" << std::endl;
                break;
            case mavlink_ctrl::ConnectionState::Connected:
                std::cout << "[STATE] Connected (waiting for remote heartbeat...)" << std::endl;
                break;
            case mavlink_ctrl::ConnectionState::LinkUp:
                std::cout << "[STATE] Link Up! Remote system detected." << std::endl;
                break;
        }
    });

    controller.setAutoReconnect(true, 2000);

    std::cout << "Connecting to " << port << " @ " << baud << " baud..." << std::endl;
    if (!controller.connect(port, baud)) {
        std::cerr << "Failed to open port " << port << std::endl;
        return 1;
    }

    std::cout << "Connected! Sending GCS heartbeats at 1Hz." << std::endl;
    std::cout << "Press Ctrl+C to exit.\n" << std::endl;

    while (g_running) {
        auto t = controller.getTelemetry();

        std::cout << "--- Telemetry Snapshot ---" << std::endl;

        // Remote system info
        if (t.last_heartbeat_ms > 0) {
            std::cout << "  Remote: SysID=" << (int)t.remote_system_id
                      << " CompID=" << (int)t.remote_component_id
                      << " Type=" << (int)t.remote_type
                      << " Status=" << (int)t.remote_system_status << std::endl;
        } else {
            std::cout << "  Remote: No heartbeat received yet" << std::endl;
        }

        // Battery
        if (t.battery_voltage_mv > 0) {
            std::cout << "  Battery: " << std::fixed << std::setprecision(1) 
                      << (t.battery_voltage_mv / 1000.0f) << "V"
                      << " | " << (t.battery_current_ca / 100.0f) << "A"
                      << " | " << (int)t.battery_remaining << "%" << std::endl;
        }

        // Attitude
        if (t.attitude_roll != 0 || t.attitude_pitch != 0 || t.attitude_yaw != 0) {
            std::cout << "  Attitude: R=" << std::fixed << std::setprecision(1) 
                      << (t.attitude_roll * 57.2958f) << "°"
                      << " P=" << (t.attitude_pitch * 57.2958f) << "°"
                      << " Y=" << (t.attitude_yaw * 57.2958f) << "°" << std::endl;
        }

        // Radio link
        if (t.radio_rssi > 0) {
            std::cout << "  Radio: RSSI=" << (int)t.radio_rssi
                      << " RemRSSI=" << (int)t.radio_remrssi
                      << " TxBuf=" << (int)t.radio_txbuf << "%"
                      << " RxErr=" << t.radio_rxerrors << std::endl;
        }

        // GPS
        if (t.gps_fix_type > 0) {
            std::cout << "  GPS: Fix=" << (int)t.gps_fix_type
                      << " Sats=" << (int)t.gps_satellites
                      << " Lat=" << (t.gps_lat / 1e7) 
                      << " Lon=" << (t.gps_lon / 1e7) << std::endl;
        }

        // Named values (servo encoder angles from Raspberry Pi)
        if (!t.named_values.empty()) {
            std::cout << "  Named Values:" << std::endl;
            for (const auto& [name, value] : t.named_values) {
                std::cout << "    " << name << " = " << std::fixed << std::setprecision(2) << value << std::endl;
            }
        }

        // Status text
        if (!t.last_statustext.empty()) {
            std::cout << "  StatusText: [" << (int)t.last_statustext_severity << "] " << t.last_statustext << std::endl;
        }

        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    std::cout << "Shutting down..." << std::endl;
    controller.disconnect();
    return 0;
}

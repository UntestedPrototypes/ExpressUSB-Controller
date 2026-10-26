#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <iomanip>

#include "crsf/crsf_controller.h"
#include "crsf/crsf_protocol.h"

static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    
    // Parse args
    std::string port;
    unsigned int baud = 921600;
    
    if (argc < 2) {
        // List available ports
        std::cout << "Usage: " << argv[0] << " <port> [baud_rate]" << std::endl;
        std::cout << "\nAvailable serial ports:" << std::endl;
        auto ports = crsf::CrsfController::listSerialPorts();
        if (ports.empty()) {
            std::cout << "  (none found)" << std::endl;
        } else {
            for (const auto& p : ports) {
                std::cout << "  " << p << std::endl;
            }
        }
        return 1;
    }
    
    port = argv[1];
    if (argc >= 3) baud = std::stoul(argv[2]);
    
    crsf::CrsfController controller;
    
    // Set up callbacks
    controller.onStateChange([](crsf::ConnectionState state) {
        switch (state) {
            case crsf::ConnectionState::Disconnected:
                std::cout << "[STATE] Disconnected" << std::endl;
                break;
            case crsf::ConnectionState::TxConnected:
                std::cout << "[STATE] TX Module Connected (no receiver)" << std::endl;
                break;
            case crsf::ConnectionState::LinkUp:
                std::cout << "[STATE] Link Up! Receiver connected." << std::endl;
                break;
            case crsf::ConnectionState::WaitingForPort:
                std::cout << "[STATE] Waiting for the module to be plugged in..." << std::endl;
                break;
        }
    });
    
    controller.onParametersLoaded([]() {
        std::cout << "[INFO] All parameters loaded!" << std::endl;
    });
    
    // Configure before connecting
    controller.setAutoReconnect(true, 3000);
    
    // Connect
    std::cout << "Connecting to " << port << " @ " << baud << " baud..." << std::endl;
    if (!controller.connect(port, baud)) {
        std::cerr << "Failed to open port " << port << std::endl;
        return 1;
    }
    
    // Set all channels to center
    controller.setAllChannelsMid();
    
    std::cout << "Connected. Press Ctrl+C to exit." << std::endl;
    std::cout << "Telemetry will print every 2 seconds." << std::endl;
    
    // Main loop - print telemetry periodically
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        auto state = controller.getConnectionState();
        auto telem = controller.getTelemetry();
        
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        
        std::cout << "\n=== Telemetry [" << std::put_time(std::localtime(&now_c), "%H:%M:%S") << "] ==="  << std::endl;
        std::cout << "Connection: " 
                  << (state == crsf::ConnectionState::LinkUp ? "LINK UP" :
                      state == crsf::ConnectionState::TxConnected ? "TX ONLY" : "DISCONNECTED")
                  << std::endl;
        
        if (!telem.device_name.empty()) {
            std::cout << "Device: " << telem.device_name << std::endl;
        }
        
        std::cout << "Link Quality: " << (int)telem.uplink_lq << "%" << std::endl;
        std::cout << "RSSI: " << -(int)telem.uplink_rssi_1 << " dBm" << std::endl;
        std::cout << "RF Mode: " << (int)telem.rf_mode << std::endl;
        std::cout << "TX Power: " << (int)telem.uplink_tx_power << std::endl;
        
        if (telem.battery_voltage > 0) {
            std::cout << "Battery: " << (telem.battery_voltage / 10.0) << "V  "
                      << (telem.battery_current / 10.0) << "A  "
                      << (int)telem.battery_remaining << "%" << std::endl;
        }
        
        if (!telem.cells_values.empty()) {
            std::cout << "Cell Voltages: ";
            for (size_t i = 0; i < telem.cells_values.size(); ++i) {
                std::cout << telem.cells_values[i] << "mV ";
            }
            std::cout << std::endl;
        }
        
        if (telem.channels_status_present) {
            std::cout << "Arm State: " << (telem.is_armed ? "ARMED" : "DISARMED") 
                      << " (Status byte: 0x" << std::hex << (int)telem.channels_status << std::dec << ")" << std::endl;
        }
        
        if (telem.elrs_pkts_good > 0 || telem.elrs_pkts_bad > 0) {
            std::cout << "ELRS Status: " << telem.elrs_pkts_good << " Good / " 
                      << (int)telem.elrs_pkts_bad << " Bad (Flags: 0x" 
                      << std::hex << (int)telem.elrs_flags << std::dec << ")" << std::endl;
            if (!telem.elrs_status_msg.empty()) {
                std::cout << "ELRS Msg: " << telem.elrs_status_msg << std::endl;
            }
        }
        
        // Print parameters if loaded
        if (controller.areParametersLoaded()) {
            auto params = controller.getParameters();
            std::cout << "\nParameters (" << params.size() << " total):" << std::endl;
            for (const auto& p : params) {
                std::cout << "  [" << (int)p.index << "] " << p.label;
                if (p.hidden) std::cout << " (hidden)";
                if (p.select) {
                    std::cout << " = ";
                    if (p.select->current_index < p.select->options.size()) {
                        std::cout << p.select->options[p.select->current_index];
                    } else {
                        std::cout << "<invalid_index>";
                    }
                    std::cout << " (" << (int)p.select->current_index << ")";
                } else if (p.numeric) {
                    std::cout << " = " << p.numeric->value;
                    if (!p.numeric->units.empty()) std::cout << " " << p.numeric->units;
                } else if (p.string_val) {
                    std::cout << " = \"" << p.string_val->value << "\"";
                }
                std::cout << std::endl;
            }
        }
    }
    
    std::cout << "\nDisconnecting..." << std::endl;
    controller.disconnect();
    std::cout << "Done." << std::endl;
    
    return 0;
}

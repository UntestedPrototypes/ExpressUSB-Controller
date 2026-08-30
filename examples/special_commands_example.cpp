#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "crsf/crsf_controller.h"

std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    g_running = false;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: special_commands_example <COM_PORT> [BAUD]" << std::endl;
        std::cerr << "Example: special_commands_example COM10 921600" << std::endl;
        return 1;
    }

    std::string port = argv[1];
    uint32_t baud = (argc >= 3) ? std::stoul(argv[2]) : 921600;

    std::signal(SIGINT, signalHandler);

    crsf::CrsfController controller;
    
    std::cout << "Connecting to " << port << " @ " << baud << " baud..." << std::endl;
    if (!controller.connect(port, baud)) {
        std::cerr << "Failed to open port " << port << std::endl;
        return 1;
    }

    std::cout << "Connected! Waiting for module parameters to load..." << std::endl;
    
    // We register a callback that fires automatically once the CRSF controller 
    // has discovered and downloaded all the configuration menus from the ELRS module.
    std::atomic<bool> parameters_ready{false};
    controller.onParametersLoaded([&parameters_ready]() {
        parameters_ready = true;
    });

    // Wait in the main thread for the background loading to finish
    while (g_running && !parameters_ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (parameters_ready && g_running) {
        std::cout << "\n[SUCCESS] All parameters loaded!" << std::endl;
        
        auto params = controller.getParameters();
        std::vector<std::string> commands;
        
        std::cout << "\n=== Available Special Commands ===" << std::endl;
        for (const auto& p : params) {
            // In CRSF, type 13 indicates a COMMAND parameter (an executable action)
            if (p.type == 13) { 
                commands.push_back(p.label);
                std::cout << commands.size() << ". " << p.label << std::endl;
            }
        }
        
        if (commands.empty()) {
            std::cout << "No commands found on this module." << std::endl;
        } else {
            std::cout << "\nEnter the number of the command to execute (0 to exit): ";
            int choice = 0;
            if (std::cin >> choice && choice > 0 && choice <= commands.size()) {
                std::string selected = commands[choice - 1];
                std::cout << "Executing '" << selected << "'..." << std::endl;
                
                if (controller.executeCommand(selected)) {
                    std::cout << "Command sent successfully! Check your TX module." << std::endl;
                } else {
                    std::cout << "Failed to send command." << std::endl;
                }
            } else {
                std::cout << "Exiting without sending commands." << std::endl;
            }
        }
    }

    std::cout << "\nShutting down..." << std::endl;
    controller.disconnect();
    return 0;
}

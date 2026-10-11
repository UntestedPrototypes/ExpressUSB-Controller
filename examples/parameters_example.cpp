// Interactive parameter menu: lists every parameter the ELRS module exposes (the same menu as the
// Lua script), lets you browse folders and change values from the console.
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "crsf/crsf_controller.h"

using namespace crsf;

static std::string readLine() {
    std::string s;
    std::getline(std::cin, s);
    return s;
}

static bool toInt(const std::string& s, long long& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    out = std::strtoll(s.c_str(), &end, 10);
    return end && *end == '\0';
}

static std::string withUnits(const std::string& v, const std::string& units) {
    return units.empty() ? v : v + " " + units;
}

static std::string valueText(const Parameter& p) {
    switch (p.type) {
        case 0: case 1: case 2: case 3: case 4: case 5:
            if (p.numeric) return withUnits(std::to_string(p.numeric->value), p.numeric->units);
            break;
        case 8:
            if (p.numeric) {
                double v = double(p.numeric->value);
                for (int i = 0; i < p.numeric->decimal_places; ++i) v /= 10.0;
                return withUnits(std::to_string(v), p.numeric->units);
            }
            break;
        case 9:
            if (p.select && p.select->current_index < p.select->options.size())
                return p.select->options[p.select->current_index];
            break;
        case 10: case 12:
            if (p.string_val) return p.string_val->value;
            break;
    }
    return "";
}

// Edit one parameter. Returns true if a value was sent to the module.
static bool edit(CrsfController& ctl, const Parameter& p) {
    switch (p.type) {
        case 9: {
            if (!p.select) return false;
            for (size_t i = 0; i < p.select->options.size(); ++i)
                if (!p.select->options[i].empty())
                    std::cout << "   " << i << ") " << p.select->options[i]
                              << (i == p.select->current_index ? "   <- current" : "") << "\n";
            std::cout << "Option number (blank to cancel): ";
            long long n;
            if (!toInt(readLine(), n) || n < p.select->min_val || n > p.select->max_val) return false;
            uint8_t v = uint8_t(n);
            ctl.writeParameter(p.index, &v, 1);
            return true;
        }
        case 0: case 1: case 2: case 3: {
            if (!p.numeric) return false;
            std::cout << "New value " << p.numeric->min_val << ".." << p.numeric->max_val << " (blank to cancel): ";
            long long n;
            if (!toInt(readLine(), n) || n < p.numeric->min_val || n > p.numeric->max_val) return false;
            if (p.type <= 1) {
                uint8_t v = uint8_t(n);
                ctl.writeParameter(p.index, &v, 1);
            } else {
                const uint16_t w = uint16_t(n);
                const uint8_t v[2] = {uint8_t(w >> 8), uint8_t(w & 0xFF)};
                ctl.writeParameter(p.index, v, 2);
            }
            return true;
        }
        case 10: {
            std::cout << "New text, max " << (p.string_val ? int(p.string_val->max_length) : 0)
                      << " chars (blank to cancel): ";
            const std::string s = readLine();
            if (s.empty()) return false;
            std::vector<uint8_t> v(s.begin(), s.end());
            v.push_back(0);
            ctl.writeParameter(p.index, v.data(), v.size());
            return true;
        }
        case 13: {
            std::cout << "Run \"" << p.label << "\"? (y/N): ";
            if (readLine() != "y") return false;
            ctl.executeCommand(p.label);
            return true;
        }
        default:
            std::cout << "This parameter type (" << int(p.type) << ") can't be edited here.\n";
            return false;
    }
}

static void waitForParameters(CrsfController& ctl) {
    const auto start = std::chrono::steady_clock::now();
    while (!ctl.areParametersLoaded()) {
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(20)) {
            std::cout << "(timed out; showing what was received)\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: parameters_example <COM_PORT> [BAUD]\nPorts found:";
        for (const auto& p : CrsfController::listSerialPorts()) std::cerr << " " << p;
        std::cerr << "\n";
        return 1;
    }
    const std::string port = argv[1];
    const unsigned baud = argc >= 3 ? unsigned(std::stoul(argv[2])) : 921600u;

    CrsfController ctl;
    std::cout << "Connecting to " << port << " @ " << baud << "...\n";
    if (!ctl.connect(port, baud)) {
        std::cerr << "Failed to open " << port << "\n";
        return 1;
    }
    std::cout << "Loading parameters...\n";
    waitForParameters(ctl);

    std::vector<int> path;   // folder indices from the root
    for (;;) {
        const int folder = path.empty() ? 0 : path.back();
        const std::vector<Parameter> all = ctl.getParameters();
        std::vector<Parameter> items;
        for (const auto& p : all)
            if (p.parent == folder && !p.hidden && p.index != 0) items.push_back(p);
        std::sort(items.begin(), items.end(), [](const Parameter& a, const Parameter& b) { return a.index < b.index; });

        std::string title = "Root";
        for (int f : path)
            for (const auto& p : all)
                if (p.index == f) title += " / " + p.label;
        const auto t = ctl.getTelemetry();
        std::cout << "\n=== " << title << " ===  [" << (t.device_name.empty() ? "?" : t.device_name) << ", "
                  << (ctl.getConnectionState() == ConnectionState::LinkUp ? "link up" : "no receiver") << "]\n";
        for (size_t i = 0; i < items.size(); ++i) {
            const auto& p = items[i];
            std::cout << " " << (i + 1) << ". " << p.label;
            if (p.type == 11) std::cout << " >";
            else if (p.type == 13) std::cout << "  [command]";
            else std::cout << ": " << valueText(p);
            std::cout << "\n";
        }
        if (items.empty()) std::cout << " (empty)\n";
        std::cout << "\nNumber = open/change, b = back, r = reload, q = quit > ";

        const std::string in = readLine();
        if (!std::cin || in == "q") break;
        if (in == "b") { if (!path.empty()) path.pop_back(); continue; }
        if (in == "r") { ctl.reloadParameters(); waitForParameters(ctl); continue; }

        long long n;
        if (!toInt(in, n) || n < 1 || n > (long long)items.size()) { std::cout << "?\n"; continue; }
        const Parameter& p = items[size_t(n - 1)];
        if (p.type == 11) { path.push_back(p.index); continue; }
        if (p.type == 12) { std::cout << p.label << ": " << valueText(p) << "\n"; continue; }
        if (edit(ctl, p)) {
            // A change can alter other options (e.g. the packet rate list), so re-read everything.
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            ctl.reloadParameters();
            waitForParameters(ctl);
        }
    }
    ctl.disconnect();
    return 0;
}

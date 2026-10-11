#pragma once
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <array>
#include <cstdint>
#include <optional>
#include <memory>

#include "crsf/crsf_protocol.h"
#include "crsf/telemetry.h"
#include "crsf/parameters.h"

namespace crsf {

enum class ConnectionState {
    Disconnected,   // No serial connection
    TxConnected,    // Serial open, TX module responding, but no RF link to receiver
    LinkUp          // TX connected AND receiver connected (LQ > 0)
};

class CrsfController {
public:
    explicit CrsfController();
    ~CrsfController();
    
    // Connection
    bool connect(const std::string& port, unsigned int baud_rate = 921600);
    void disconnect();
    ConnectionState getConnectionState() const;
    bool isConnected() const; // true if at least TxConnected
    
    // RC Channels (values in CRSF raw: 172-1811, center=992)
    void setChannel(uint8_t index, uint16_t value);
    void setChannels(const uint16_t values[16]);
    void setAllChannelsMid(); // Set all 16 channels to 992 (center)
    
    // Telemetry
    TelemetryData getTelemetry() const;
    
    // Parameters
    std::vector<Parameter> getParameters() const;
    std::optional<Parameter> getParameter(uint8_t index) const;
    void writeParameter(uint8_t index, const uint8_t* value, size_t len);
    bool areParametersLoaded() const;
    // Discard the cached parameters and read them all again (e.g. after a write changed other options)
    void reloadParameters();
    
    // Common ExpressLRS Special Actions
    enum class ElrsAction {
        Bind,
        WiFiConnectivity,
        BleJoystick,
        VtxAdministrator,
        ResetVtx,
        EnterBootloader
    };

    // Commands
    // Trigger a specific command parameter on the module by known enum Action
    bool executeAction(ElrsAction action);
    
    // Trigger a specific command parameter on the module by string name (e.g. "Bind")
    bool executeCommand(const std::string& command_name);
    
    // Convenience function to trigger Bind mode
    bool triggerBind();
    
    // Manual operations
    void ping();
    
    // Configuration
    void setHeartbeatInterval(unsigned int microseconds); // Default 4000 (250Hz)
    void setAutoReconnect(bool enabled, unsigned int interval_ms = 2000);
    
    // Callbacks
    using StateCallback = std::function<void(ConnectionState)>;
    using VoidCallback = std::function<void()>;
    void onStateChange(StateCallback cb);
    void onParametersLoaded(VoidCallback cb);
    
    // Utility: list available serial ports (platform-specific)
    static std::vector<std::string> listSerialPorts();
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace crsf

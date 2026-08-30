#pragma once
#include <string>
#include <cstdint>
#include <functional>
#include <memory>
#include <common/mavlink.h>
#include "mavlink_controller/telemetry.h"

namespace mavlink_ctrl {

enum class ConnectionState {
    Disconnected,
    Connected,    // Serial port open, no remote heartbeat yet
    LinkUp        // Remote heartbeat received
};

class MavlinkController {
public:
    MavlinkController();
    ~MavlinkController();

    // Connection
    bool connect(const std::string& port, uint32_t baud = 460800);
    void disconnect();
    bool isConnected() const;
    ConnectionState getState() const;

    // RC Control (PWM 1000-2000µs, UINT16_MAX to ignore, 0 to release)
    void setChannel(uint8_t index, uint16_t value_us);
    void setChannels(const uint16_t channels[18]);

    // Telemetry
    TelemetryData getTelemetry() const;

    // Named values (convenience accessor)
    float getNamedValue(const std::string& name, float default_val = 0.0f) const;

    // Callbacks
    void onStateChange(std::function<void(ConnectionState)> cb);
    void onMessage(std::function<void(const mavlink_message_t&)> cb);

    // Configuration - call before connect()
    void setTargetSystem(uint8_t system_id, uint8_t component_id = 1);
    void setGcsIds(uint8_t system_id = 255, uint8_t component_id = 190);
    void setHeartbeatRate(uint32_t interval_ms = 1000);
    void setRcRate(uint32_t interval_us = 20000);
    void setAutoReconnect(bool enable, uint32_t interval_ms = 2000);

    // Utility
    static std::vector<std::string> listSerialPorts();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mavlink_ctrl

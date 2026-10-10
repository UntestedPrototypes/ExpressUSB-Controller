#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include <common/mavlink.h>
#include "mavlink_controller/link_protocol.h"
#include "mavlink_controller/telemetry.h"
#include "mavlink_controller/transport.h"

namespace mavlink_ctrl {

enum class ConnectionState {
    Disconnected,
    Connected,    // Serial port open, remote end not heard from yet
    LinkUp        // MAVLink: remote heartbeat received. CRSF: link statistics with a link quality above 0
};

// Talks to an ExpressLRS TX module over its serial/USB port, in one of two protocols (see LinkMode):
//   Mavlink : the module runs in Native MAVLink mode. Default, 460800 baud.
//   Crsf    : the module runs in CRSF mode and the PC is its handset. Default, 400000 baud.
// The same calls (setChannel, getTelemetry, onStateChange ...) work in both modes.
class MavlinkController {
public:
    MavlinkController();
    ~MavlinkController();

    // Link mode: choose before connect(). Default Mavlink. Switch by disconnect(), setLinkMode(), connect().
    // The TX module (and receiver) must be set to the same mode in ExpressLRS.
    void setLinkMode(LinkMode mode);
    LinkMode getLinkMode() const;
    static uint32_t defaultBaud(LinkMode mode);                  // 460800 for Mavlink, 400000 for Crsf
    static const char* linkModeName(LinkMode mode);              // "mavlink" / "crsf"
    static bool parseLinkMode(const std::string& text, LinkMode& out);  // case-insensitive

    // Connection. baud 0 = the default of the link mode.
    bool connect(const std::string& port, uint32_t baud = 0);
    void disconnect();
    bool isConnected() const;
    ConnectionState getState() const;

    // RC Control: microseconds, 1000-2000 (1500 = centre).
    //   Mavlink: RC_CHANNELS_OVERRIDE, 18 channels. UINT16_MAX = ignore (the module passes the radio's
    //            own value), 0 = release.
    //   Crsf   : RC channels frame, 16 channels (17 and 18 are ignored). There is no other source, so
    //            UINT16_MAX and 0 both mean "keep the last value" (centred until a value was set).
    void setChannel(uint8_t index, uint16_t value_us);
    void setChannels(const uint16_t channels[18]);

    // Telemetry
    TelemetryData getTelemetry() const;

    // Named values (convenience accessor, MAVLink mode)
    float getNamedValue(const std::string& name, float default_val = 0.0f) const;

    // Callbacks
    void onStateChange(std::function<void(ConnectionState)> cb);
    void onMessage(std::function<void(const mavlink_message_t&)> cb);   // Mavlink mode: every MAVLink message
    // Crsf mode: every valid CRSF frame (type, payload without the type and crc)
    void onCrsfFrame(std::function<void(uint8_t type, const uint8_t* payload, size_t length)> cb);

    // Configuration - call before connect()
    void setTargetSystem(uint8_t system_id, uint8_t component_id = 1);       // Mavlink
    void setGcsIds(uint8_t system_id = 255, uint8_t component_id = 190);     // Mavlink
    void setHeartbeatRate(uint32_t interval_ms = 1000);                      // Mavlink (CRSF needs no heartbeat)
    // RC frame interval. Default: 20000 us (50 Hz) in Mavlink mode, 4000 us (250 Hz) in Crsf mode.
    void setRcRate(uint32_t interval_us = 20000);
    void setAutoReconnect(bool enable, uint32_t interval_ms = 2000);

    // Advanced: use another byte transport instead of the serial port (self-tests, adapters).
    // Call before connect().
    void setTransport(std::unique_ptr<ITransport> transport);

    // Utility
    static std::vector<std::string> listSerialPorts();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mavlink_ctrl

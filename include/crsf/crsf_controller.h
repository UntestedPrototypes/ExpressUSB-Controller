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
    LinkUp,         // TX connected AND receiver connected (LQ > 0)
    WaitingForPort  // connectWhenAvailable(): watching for the port to appear (module not plugged in yet)
};

// startConnect(): where the reboot-and-connect sequence is
enum class ConnectStep {
    Rebooting,       // pulsing the module's reset line
    WaitingForBoot,  // giving the module time to start
    WaitingForPort,  // port not there (yet): module unplugged
    Connecting,      // port open, asking the module to identify itself
    Connected,       // done: the module answered (final)
    NotFound,        // gave up after all tries (final)
    Cancelled        // disconnect() was called (final)
};

struct ConnectProgress {
    ConnectStep step;
    int attempt = 0;        // 1-based try number for WaitingForPort / Connecting, 0 otherwise
    int attempts = 0;       // total tries
    std::string message;    // ready-to-show text, e.g. "Connecting to the module (try 2/3)..."
};

struct ConnectOptions {
    bool reboot = true;                    // reset the module first (needs the RTS/DTR auto-reset wiring)
    unsigned int boot_wait_ms = 2000;      // after the reset, before the first try
    int attempts = 3;
    unsigned int attempt_timeout_ms = 3000;  // per try: how long to wait for the module to answer
    unsigned int retry_delay_ms = 1000;      // between tries
};

class CrsfController {
public:
    explicit CrsfController();
    ~CrsfController();
    
    // Connection
    bool connect(const std::string& port, unsigned int baud_rate = 921600);
    // Connects now if the port is there, otherwise watches for it and connects the moment it appears, so the
    // module can be plugged in after the app has started (an ELRS TX that sits unconnected after power-up
    // switches to WiFi mode after a while). State is WaitingForPort until then. Stopped by disconnect().
    // Returns false only if already connected or waiting.
    bool connectWhenAvailable(const std::string& port, unsigned int baud_rate = 921600, unsigned int poll_ms = 250);
    // Reboots the module, waits for it to start, then tries to connect up to `attempts` times, waiting each time
    // until the module has identified itself. Runs in the background and reports every step through `progress`
    // (called from a background thread), ending with Connected, NotFound or Cancelled. Starting from a reboot
    // also brings back a module that has gone into WiFi mode. Returns false if already connected or busy.
    using ProgressCallback = std::function<void(const ConnectProgress&)>;
    bool startConnect(const std::string& port, unsigned int baud_rate = 921600,
                      const ConnectOptions& options = {}, ProgressCallback progress = {});
    bool isConnecting() const;
    // Just the reboot (port must not be open). False if the port can't be opened.
    static bool rebootModule(const std::string& port);

    void disconnect();          // also stops connectWhenAvailable() / startConnect()
    ConnectionState getConnectionState() const;
    bool isConnected() const;   // true if at least TxConnected
    bool isWaitingForPort() const;
    // Milliseconds since the last valid frame from the module, -1 if none since connecting. A module that
    // stops talking (unplugged, or gone into WiFi mode) shows up here while the port is still open.
    int64_t millisecondsSinceLastFrame() const;
    
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
    void closeConnection();     // disconnect() without stopping the background connect helpers
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace crsf

#include "mavlink_controller/mavlink_controller.h"
#include "mavlink_controller/serial_port.h"
#include "mavlink_controller/mavlink_handler.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <cstring>
#endif

namespace mavlink_ctrl {

struct MavlinkController::Impl {
    SerialPort serial_;
    Telemetry telemetry_;
    MavlinkHandler handler_{telemetry_};

    // Connection state
    std::atomic<ConnectionState> state_{ConnectionState::Disconnected};
    std::function<void(ConnectionState)> state_cb_;
    std::function<void(const mavlink_message_t&)> message_cb_;

    // RC channels
    std::mutex channels_mutex_;
    uint16_t channels_[18];

    // Heartbeat thread
    std::thread heartbeat_thread_;
    uint32_t heartbeat_interval_ms_ = 1000;

    // RC thread
    std::thread rc_thread_;
    uint32_t rc_interval_us_ = 20000;

    // Auto-reconnect
    std::thread reconnect_thread_;
    bool auto_reconnect_ = false;
    uint32_t reconnect_interval_ms_ = 2000;
    std::string port_name_;
    uint32_t baud_rate_ = 460800;

    std::atomic<bool> running_{false};

    Impl() {
        // Initialize all channels to UINT16_MAX (ignore/passthrough)
        for (auto& ch : channels_) ch = UINT16_MAX;
    }

    void setState(ConnectionState new_state) {
        auto old = state_.exchange(new_state);
        if (old != new_state && state_cb_) {
            state_cb_(new_state);
        }
    }

    void updateState() {
        if (!serial_.isOpen()) {
            setState(ConnectionState::Disconnected);
            return;
        }

        if (handler_.hasRemoteHeartbeat()) {
            setState(ConnectionState::LinkUp);
        } else {
            setState(ConnectionState::Connected);
        }
    }

    void heartbeatLoop() {
        while (running_) {
            auto frame = handler_.buildHeartbeat();
            serial_.writeCommand(frame);
            
            // Check connection state after sending heartbeat
            updateState();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
        }
    }

    void rcLoop() {
        while (running_) {
            uint16_t ch[18];
            {
                std::lock_guard<std::mutex> lock(channels_mutex_);
                std::copy(std::begin(channels_), std::end(channels_), std::begin(ch));
            }

            auto frame = handler_.buildRcOverride(ch);
            serial_.writeRC(frame);

            std::this_thread::sleep_for(std::chrono::microseconds(rc_interval_us_));
        }
    }

    void reconnectLoop() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
            
            if (!running_) break;
            
            if (!serial_.isOpen() && auto_reconnect_) {
                std::cout << "[MAVLink] Attempting reconnection to " << port_name_ << "...\n";
                telemetry_.reset();
                
                if (serial_.open(port_name_, baud_rate_)) {
                    setState(ConnectionState::Connected);
                } else {
                    setState(ConnectionState::Disconnected);
                }
            }
        }
    }
};

MavlinkController::MavlinkController() : impl_(std::make_unique<Impl>()) {}

MavlinkController::~MavlinkController() {
    disconnect();
}

bool MavlinkController::connect(const std::string& port, uint32_t baud) {
    if (impl_->running_) disconnect();

    impl_->port_name_ = port;
    impl_->baud_rate_ = baud;

    // Set up serial receive callback -> feed bytes to MAVLink parser
    impl_->serial_.setReceiveCallback([this](const uint8_t* data, size_t len) {
        impl_->handler_.processBytes(data, len);
    });

    // Set up raw message callback if user registered one
    if (impl_->message_cb_) {
        impl_->handler_.setMessageCallback(impl_->message_cb_);
    }

    if (!impl_->serial_.open(port, baud)) {
        return false;
    }

    impl_->running_ = true;
    impl_->setState(ConnectionState::Connected);

    // Start heartbeat thread (1Hz by default)
    impl_->heartbeat_thread_ = std::thread([this]() {
        impl_->heartbeatLoop();
    });

    // Start RC override thread (50Hz by default)
    impl_->rc_thread_ = std::thread([this]() {
        impl_->rcLoop();
    });

    // Start auto-reconnect thread if enabled
    if (impl_->auto_reconnect_) {
        impl_->reconnect_thread_ = std::thread([this]() {
            impl_->reconnectLoop();
        });
    }

    return true;
}

void MavlinkController::disconnect() {
    impl_->running_ = false;

    if (impl_->heartbeat_thread_.joinable()) impl_->heartbeat_thread_.join();
    if (impl_->rc_thread_.joinable()) impl_->rc_thread_.join();
    if (impl_->reconnect_thread_.joinable()) impl_->reconnect_thread_.join();

    impl_->serial_.close();
    impl_->setState(ConnectionState::Disconnected);
}

bool MavlinkController::isConnected() const {
    return impl_->state_ != ConnectionState::Disconnected;
}

ConnectionState MavlinkController::getState() const {
    return impl_->state_;
}

void MavlinkController::setChannel(uint8_t index, uint16_t value_us) {
    if (index >= 18) return;
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    impl_->channels_[index] = value_us;
}

void MavlinkController::setChannels(const uint16_t channels[18]) {
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    std::copy(channels, channels + 18, impl_->channels_);
}

TelemetryData MavlinkController::getTelemetry() const {
    return impl_->telemetry_.get();
}

float MavlinkController::getNamedValue(const std::string& name, float default_val) const {
    auto data = impl_->telemetry_.get();
    auto it = data.named_values.find(name);
    if (it != data.named_values.end()) return it->second;
    return default_val;
}

void MavlinkController::onStateChange(std::function<void(ConnectionState)> cb) {
    impl_->state_cb_ = std::move(cb);
}

void MavlinkController::onMessage(std::function<void(const mavlink_message_t&)> cb) {
    impl_->message_cb_ = std::move(cb);
    impl_->handler_.setMessageCallback(cb);
}

void MavlinkController::setTargetSystem(uint8_t system_id, uint8_t component_id) {
    impl_->handler_.setTargetIds(system_id, component_id);
}

void MavlinkController::setGcsIds(uint8_t system_id, uint8_t component_id) {
    impl_->handler_.setGcsIds(system_id, component_id);
}

void MavlinkController::setHeartbeatRate(uint32_t interval_ms) {
    impl_->heartbeat_interval_ms_ = interval_ms;
}

void MavlinkController::setRcRate(uint32_t interval_us) {
    impl_->rc_interval_us_ = interval_us;
}

void MavlinkController::setAutoReconnect(bool enable, uint32_t interval_ms) {
    impl_->auto_reconnect_ = enable;
    impl_->reconnect_interval_ms_ = interval_ms;
}

std::vector<std::string> MavlinkController::listSerialPorts() {
    std::vector<std::string> ports;

#ifdef _WIN32
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        char valueName[256], valueData[256];
        DWORD valueNameSize, valueDataSize, type;
        DWORD index = 0;

        while (true) {
            valueNameSize = sizeof(valueName);
            valueDataSize = sizeof(valueData);
            LONG result = RegEnumValueA(hKey, index, valueName, &valueNameSize, nullptr, &type, (LPBYTE)valueData, &valueDataSize);
            if (result != ERROR_SUCCESS) break;

            if (type == REG_SZ) {
                std::string port = valueData;
                // Support high COM ports (>= COM10) by using UNC path
                if (port.size() > 4 || (port.size() == 4 && port[3] >= '1')) {
                    port = "\\\\.\\" + port;
                }
                ports.push_back(port);
            }
            index++;
        }
        RegCloseKey(hKey);
    }
#else
    // Linux: scan /dev for common serial device names
    const char* patterns[] = {"ttyUSB", "ttyACM", "ttyS"};
    DIR* dir = opendir("/dev");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            for (const char* pattern : patterns) {
                if (std::strncmp(entry->d_name, pattern, std::strlen(pattern)) == 0) {
                    ports.push_back(std::string("/dev/") + entry->d_name);
                    break;
                }
            }
        }
        closedir(dir);
    }
#endif

    return ports;
}

} // namespace mavlink_ctrl

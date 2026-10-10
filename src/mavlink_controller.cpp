#include "mavlink_controller/mavlink_controller.h"
#include "mavlink_controller/serial_port.h"
#include "mavlink_controller/mavlink_handler.h"
#include "mavlink_controller/crsf_handler.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#include <cstring>
#endif

namespace mavlink_ctrl {

struct MavlinkController::Impl {
    std::unique_ptr<ITransport> transport_ = std::make_unique<SerialPort>();
    Telemetry telemetry_;

    // Both protocols exist; the one selected by the link mode is active while connected.
    MavlinkHandler mavlink_{telemetry_};
    CrsfHandler crsf_{telemetry_};
    LinkMode mode_ = LinkMode::Mavlink;
    LinkProtocol* protocol_ = &mavlink_;

    // Connection state
    std::atomic<ConnectionState> state_{ConnectionState::Disconnected};
    std::function<void(ConnectionState)> state_cb_;
    std::function<void(const mavlink_message_t&)> message_cb_;

    // RC channels
    std::mutex channels_mutex_;
    uint16_t channels_[18];

    // Worker threads sleep on this, so disconnect() does not wait for a timer to run out
    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;

    // Keep-alive / state thread
    std::thread heartbeat_thread_;
    uint32_t heartbeat_interval_ms_ = 1000;

    // RC thread
    std::thread rc_thread_;
    uint32_t rc_interval_us_ = 0;   // 0 = default of the link mode

    // Auto-reconnect
    std::thread reconnect_thread_;
    bool auto_reconnect_ = false;
    uint32_t reconnect_interval_ms_ = 2000;
    std::string port_name_;
    uint32_t baud_rate_ = 0;

    std::atomic<bool> running_{false};

#ifdef _WIN32
    // Standard sleeps on Windows wake up in steps of about 15.6 ms, which would turn a 250 Hz RC stream
    // into 64 Hz. A high resolution waitable timer keeps the schedule (about 0.2 ms accuracy).
    HANDLE rc_timer_ = nullptr;
    HANDLE stop_event_ = nullptr;

    void createRcTimer() {
        destroyRcTimer();
        stop_event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);   // manual reset
        rc_timer_ = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
        if (!rc_timer_) rc_timer_ = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);   // older Windows: coarse
    }
    void destroyRcTimer() {
        if (rc_timer_) CloseHandle(rc_timer_);
        if (stop_event_) CloseHandle(stop_event_);
        rc_timer_ = stop_event_ = nullptr;
    }
#endif

    Impl() {
        // Initialize all channels to UINT16_MAX (ignore/passthrough)
        for (auto& ch : channels_) ch = UINT16_MAX;
    }

    uint32_t rcIntervalUs() const {
        if (rc_interval_us_) return rc_interval_us_;
        return mode_ == LinkMode::Crsf ? 4000 : 20000;
    }

    void setState(ConnectionState new_state) {
        auto old = state_.exchange(new_state);
        if (old != new_state && state_cb_) {
            state_cb_(new_state);
        }
    }

    void updateState() {
        if (!transport_->isOpen()) {
            setState(ConnectionState::Disconnected);
            return;
        }

        if (protocol_->linkUp()) {
            setState(ConnectionState::LinkUp);
        } else {
            setState(ConnectionState::Connected);
        }
    }

    // Returns true if the thread should stop
    template <class Duration>
    bool sleepInterruptible(Duration d) {
        std::unique_lock<std::mutex> lock(wake_mutex_);
        return wake_cv_.wait_for(lock, d, [this] { return !running_.load(); });
    }

    void heartbeatLoop() {
        auto next_keepalive = std::chrono::steady_clock::now();
        while (running_) {
            if (std::chrono::steady_clock::now() >= next_keepalive) {
                auto frame = protocol_->buildKeepAlive();   // MAVLink: GCS heartbeat. CRSF: nothing to send.
                if (!frame.empty()) transport_->writeCommand(frame);
                next_keepalive = std::chrono::steady_clock::now() + std::chrono::milliseconds(heartbeat_interval_ms_);
            }

            // The link state is re-evaluated often, so a lost link shows up quickly
            updateState();

            if (sleepInterruptible(std::chrono::milliseconds(100))) break;
        }
    }

    // Sleep until a point in time (so the RC rate does not drift with the time a frame takes).
    // Returns true if the thread should stop.
    bool sleepUntil(std::chrono::steady_clock::time_point when) {
#ifdef _WIN32
        if (rc_timer_ && stop_event_) {
            const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(when - std::chrono::steady_clock::now());
            if (remaining.count() <= 0) return !running_.load();
            LARGE_INTEGER due;
            due.QuadPart = -10 * static_cast<LONGLONG>(remaining.count());   // relative, 100 ns units
            SetWaitableTimer(rc_timer_, &due, 0, nullptr, nullptr, FALSE);
            const HANDLE handles[2] = {rc_timer_, stop_event_};
            return WaitForMultipleObjects(2, handles, FALSE, INFINITE) != WAIT_OBJECT_0;   // stop event = stop
        }
#endif
        std::unique_lock<std::mutex> lock(wake_mutex_);
        return wake_cv_.wait_until(lock, when, [this] { return !running_.load(); });
    }

    void rcLoop() {
        auto next = std::chrono::steady_clock::now();
        while (running_) {
            uint16_t ch[18];
            {
                std::lock_guard<std::mutex> lock(channels_mutex_);
                std::copy(std::begin(channels_), std::end(channels_), std::begin(ch));
            }

            auto frame = protocol_->buildRcFrame(ch);
            transport_->writeRC(frame);

            next += std::chrono::microseconds(rcIntervalUs());
            const auto now = std::chrono::steady_clock::now();
            if (next < now) next = now;   // running late (a stalled system): do not fire a burst to catch up
            if (sleepUntil(next)) break;
        }
    }

    void reconnectLoop() {
        while (running_) {
            if (sleepInterruptible(std::chrono::milliseconds(reconnect_interval_ms_))) break;

            if (!transport_->isOpen() && auto_reconnect_) {
                std::cout << "[Link] Attempting reconnection to " << port_name_ << "...\n";
                telemetry_.reset();
                protocol_->reset();

                if (transport_->open(port_name_, baud_rate_)) {
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

void MavlinkController::setLinkMode(LinkMode mode) {
    impl_->mode_ = mode;   // takes effect at the next connect()
}

LinkMode MavlinkController::getLinkMode() const {
    return impl_->mode_;
}

uint32_t MavlinkController::defaultBaud(LinkMode mode) {
    return mode == LinkMode::Crsf ? 400000u : 460800u;
}

const char* MavlinkController::linkModeName(LinkMode mode) {
    return mode == LinkMode::Crsf ? "crsf" : "mavlink";
}

bool MavlinkController::parseLinkMode(const std::string& text, LinkMode& out) {
    std::string t;
    for (char c : text) t.push_back(char(std::tolower(static_cast<unsigned char>(c))));
    if (t == "mavlink" || t == "mav") { out = LinkMode::Mavlink; return true; }
    if (t == "crsf") { out = LinkMode::Crsf; return true; }
    return false;
}

void MavlinkController::setTransport(std::unique_ptr<ITransport> transport) {
    if (impl_->running_) disconnect();
    if (transport) impl_->transport_ = std::move(transport);
}

bool MavlinkController::connect(const std::string& port, uint32_t baud) {
    if (impl_->running_) disconnect();

    impl_->protocol_ = impl_->mode_ == LinkMode::Crsf ? static_cast<LinkProtocol*>(&impl_->crsf_)
                                                      : static_cast<LinkProtocol*>(&impl_->mavlink_);
    impl_->protocol_->reset();
    impl_->telemetry_.reset();   // nothing of an earlier connection (or the other mode) stays
    if (baud == 0) baud = defaultBaud(impl_->mode_);

    impl_->port_name_ = port;
    impl_->baud_rate_ = baud;

    // Set up serial receive callback -> feed bytes to the selected protocol's parser
    impl_->transport_->setReceiveCallback([this](const uint8_t* data, size_t len) {
        impl_->protocol_->processBytes(data, len);
    });

    // Set up raw message callback if user registered one
    if (impl_->message_cb_) {
        impl_->mavlink_.setMessageCallback(impl_->message_cb_);
    }

    if (!impl_->transport_->open(port, baud)) {
        return false;
    }

#ifdef _WIN32
    impl_->createRcTimer();
#endif
    impl_->running_ = true;
    impl_->setState(ConnectionState::Connected);

    // Keep-alive + link state thread (MAVLink: GCS heartbeat, 1 Hz by default)
    impl_->heartbeat_thread_ = std::thread([this]() {
        impl_->heartbeatLoop();
    });

    // RC thread (50 Hz in MAVLink mode, 250 Hz in CRSF mode, unless setRcRate() was called)
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
    {
        std::lock_guard<std::mutex> lock(impl_->wake_mutex_);
        impl_->running_ = false;
    }
    impl_->wake_cv_.notify_all();
#ifdef _WIN32
    if (impl_->stop_event_) SetEvent(impl_->stop_event_);   // wakes the RC thread out of its timer wait
#endif

    if (impl_->heartbeat_thread_.joinable()) impl_->heartbeat_thread_.join();
    if (impl_->rc_thread_.joinable()) impl_->rc_thread_.join();
    if (impl_->reconnect_thread_.joinable()) impl_->reconnect_thread_.join();

    impl_->transport_->close();
#ifdef _WIN32
    impl_->destroyRcTimer();
#endif
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
    impl_->mavlink_.setMessageCallback(impl_->message_cb_);
}

void MavlinkController::onCrsfFrame(std::function<void(uint8_t, const uint8_t*, size_t)> cb) {
    impl_->crsf_.setFrameCallback(std::move(cb));
}

void MavlinkController::setTargetSystem(uint8_t system_id, uint8_t component_id) {
    impl_->mavlink_.setTargetIds(system_id, component_id);
}

void MavlinkController::setGcsIds(uint8_t system_id, uint8_t component_id) {
    impl_->mavlink_.setGcsIds(system_id, component_id);
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

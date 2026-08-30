#include "crsf/crsf_controller.h"
#include "crsf/serial_port.h"
#include "crsf/frame_reader.h"
#include "crsf/frame_parser.h"
#include "crsf/frame_builder.h"

#include <iostream>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <filesystem>
#endif

namespace crsf {

// Address constants as uint8_t for frame builder calls
static constexpr uint8_t ADDR_BROADCAST       = static_cast<uint8_t>(Address::BROADCAST);
static constexpr uint8_t ADDR_CRSF_TRANSMITTER = static_cast<uint8_t>(Address::CRSF_TRANSMITTER);
static constexpr uint8_t ADDR_RADIO           = static_cast<uint8_t>(Address::RADIO_TRANSMITTER);

class CrsfController::Impl {
public:
    SerialPort serial_;
    FrameReader reader_;
    Telemetry telemetry_;
    ParameterStore parameters_;
    FrameParser parser_;
    
    std::array<uint16_t, 16> channels_;
    std::mutex channels_mutex_;
    
    std::thread heartbeat_thread_;
    std::atomic<bool> running_{false};
    std::atomic<unsigned int> heartbeat_interval_us_{4000};
    
    std::atomic<ConnectionState> state_{ConnectionState::Disconnected};
    
    std::atomic<bool> auto_reconnect_{false};
    std::atomic<unsigned int> reconnect_interval_ms_{2000};
    std::thread reconnect_thread_;
    std::string current_port_;
    unsigned int current_baud_ = 921600;

    StateCallback state_cb_;
    VoidCallback params_loaded_cb_;
    
    bool params_loading_started_ = false;
    bool params_loaded_fired_ = false;

    Impl()
        : reader_({CRSF_SYNC_BYTE,
                    static_cast<uint8_t>(Address::RADIO_TRANSMITTER),
                    static_cast<uint8_t>(Address::CRSF_TRANSMITTER)})
        , parser_(telemetry_, parameters_)
    {
        std::fill(channels_.begin(), channels_.end(), CRSF_CHANNEL_MID);
        setupReceiveCallback(); // Bind receive callback permanently
    }
    
    void updateState() {
        ConnectionState new_state = ConnectionState::Disconnected;
        if (serial_.isOpen()) {
            auto telem = telemetry_.get();
            if (!telem.device_name.empty()) {
                new_state = (telem.uplink_lq > 0) 
                    ? ConnectionState::LinkUp 
                    : ConnectionState::TxConnected;
            } else {
                // Serial is open but haven't received device info yet
                new_state = ConnectionState::TxConnected;
            }
        }
        
        ConnectionState old_state = state_.exchange(new_state);
        if (new_state != old_state && state_cb_) {
            state_cb_(new_state);
        }
    }
    
    void setupReceiveCallback() {
        serial_.setReceiveCallback([this](const uint8_t* data, size_t len) {
            reader_.feed(data, len);
        });
    }
};

CrsfController::CrsfController() : impl_(std::make_unique<Impl>()) {
    // Wire up frame reader -> parser pipeline
    impl_->reader_.setFrameCallback([this](const std::vector<uint8_t>& frame) {
        impl_->parser_.parse(frame);
        
        // Auto-start parameter reading when device info first arrives
        auto telem = impl_->telemetry_.get();
        if (!telem.device_name.empty() && !impl_->params_loading_started_ && telem.parameters_count > 0) {
            impl_->params_loading_started_ = true;
            impl_->parameters_.beginReadAll(telem.parameters_count);
        }
        
        // Fire parameters loaded callback once
        if (impl_->parameters_.isReadComplete() && !impl_->params_loaded_fired_) {
            impl_->params_loaded_fired_ = true;
            if (impl_->params_loaded_cb_) {
                impl_->params_loaded_cb_();
            }
        }
        
        impl_->updateState();
    });

    // Wire up parameter read requests -> serial write
    impl_->parameters_.setReadRequestCallback([this](uint8_t param_index, uint8_t chunk_index) {
        auto frame = frame_builder::buildParameterRead(
            ADDR_CRSF_TRANSMITTER, ADDR_RADIO, param_index, chunk_index);
        impl_->serial_.writeCommand(frame);
    });
}

CrsfController::~CrsfController() {
    disconnect();
}

bool CrsfController::connect(const std::string& port, unsigned int baud_rate) {

    if (impl_->running_) return false;
    
    impl_->current_port_ = port;
    impl_->current_baud_ = baud_rate;
    
    // Reset state from any previous connection
    impl_->telemetry_.reset();
    impl_->parameters_.reset();
    impl_->reader_.reset();
    impl_->params_loading_started_ = false;
    impl_->params_loaded_fired_ = false;
    
    // Attempt to open port
    std::string actual_port = port;
#ifdef _WIN32
    // Windows requires \\.\ prefix for COM10 and above
    if (port.find("COM") == 0 && port.length() > 4) {
        actual_port = "\\\\.\\" + port;
    }
#endif

    if (!impl_->serial_.open(actual_port, baud_rate)) {
        return false;
    }
    
    impl_->running_ = true;
    
    // Start heartbeat thread (RC channel transmission)
    impl_->heartbeat_thread_ = std::thread([this]() {
        while (impl_->running_) {
            if (!impl_->serial_.isOpen()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            std::array<uint16_t, 16> channels;
            {
                std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
                channels = impl_->channels_;
            }
            
            auto frame = frame_builder::buildRCChannelsFrame(channels.data());
            impl_->serial_.writeRC(frame);
            
            std::this_thread::sleep_for(
                std::chrono::microseconds(impl_->heartbeat_interval_us_.load()));
        }
    });

    // Start auto-reconnect thread if enabled
    if (impl_->auto_reconnect_) {
        impl_->reconnect_thread_ = std::thread([this]() {
            while (impl_->running_) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(impl_->reconnect_interval_ms_.load()));
                    
                if (impl_->running_ && !impl_->serial_.isOpen()) {
                    std::cout << "[CRSF] Auto-reconnecting to " << impl_->current_port_ << "...\n";
                    impl_->serial_.close();
                    impl_->telemetry_.reset();
                    impl_->parameters_.reset();
                    impl_->reader_.reset();
                    impl_->params_loading_started_ = false;
                    impl_->params_loaded_fired_ = false;
                    
                    if (impl_->serial_.open(impl_->current_port_, impl_->current_baud_)) {
                        ping();
                        std::cout << "[CRSF] Reconnected.\n";
                    }
                    impl_->updateState();
                }
            }
        });
    }


    // Send initial device ping
    ping();
    

    // Initialize connection state correctly
    impl_->updateState();
    

    return true;
}

void CrsfController::disconnect() {
    if (!impl_->running_) return;
    
    impl_->running_ = false;
    
    if (impl_->heartbeat_thread_.joinable()) {
        impl_->heartbeat_thread_.join();
    }
    
    if (impl_->reconnect_thread_.joinable()) {
        impl_->reconnect_thread_.join();
    }
    
    impl_->serial_.close();
    impl_->telemetry_.reset();
    impl_->parameters_.reset();
    impl_->reader_.reset();
    impl_->params_loading_started_ = false;
    impl_->params_loaded_fired_ = false;
    
    impl_->state_ = ConnectionState::Disconnected;
    if (impl_->state_cb_) impl_->state_cb_(ConnectionState::Disconnected);
}

ConnectionState CrsfController::getConnectionState() const {
    return impl_->state_;
}

bool CrsfController::isConnected() const {
    return impl_->state_ != ConnectionState::Disconnected;
}

void CrsfController::setChannel(uint8_t index, uint16_t value) {
    if (index < 16) {
        std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
        impl_->channels_[index] = value & 0x07FF; // Clamp to 11 bits
    }
}

void CrsfController::setChannels(const uint16_t values[16]) {
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    for (int i = 0; i < 16; ++i) {
        impl_->channels_[i] = values[i] & 0x07FF;
    }
}

void CrsfController::setAllChannelsMid() {
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    std::fill(impl_->channels_.begin(), impl_->channels_.end(), CRSF_CHANNEL_MID);
}

TelemetryData CrsfController::getTelemetry() const {
    return impl_->telemetry_.get();
}

std::vector<Parameter> CrsfController::getParameters() const {
    return impl_->parameters_.getAll();
}

bool CrsfController::executeAction(ElrsAction action) {
    std::string command_name;
    switch (action) {
        case ElrsAction::Bind: command_name = "Bind"; break;
        case ElrsAction::WiFiConnectivity: command_name = "WiFi Connectivity"; break;
        case ElrsAction::BleJoystick: command_name = "BLE Joystick"; break;
        case ElrsAction::VtxAdministrator: command_name = "VTX Administrator"; break;
        case ElrsAction::ResetVtx: command_name = "Reset VTX"; break;
        case ElrsAction::EnterBootloader: command_name = "Enter Bootloader"; break;
        default: return false;
    }
    return executeCommand(command_name);
}

bool CrsfController::executeCommand(const std::string& command_name) {
    if (!impl_->serial_.isOpen()) return false;
    
    auto params = getParameters();
    for (const auto& p : params) {
        if (p.label == command_name) {
            // For ExpressLRS COMMAND types (like Bind, WiFi), writing a value of '1' (or '4') 
            // to the parameter index executes the command action on the TX module.
            // Value 1 usually represents l_start or execute state.
            uint8_t value = 4; // Most ELRS command routines look for 1 (start) or 4 (status/execute)
            
            auto frame = frame_builder::buildParameterWrite(
                static_cast<uint8_t>(Address::CRSF_TRANSMITTER),
                static_cast<uint8_t>(Address::RADIO_TRANSMITTER),
                p.index, &value, 1);
                
            impl_->serial_.writeCommand(frame);
            return true;
        }
    }
    return false;
}

bool CrsfController::triggerBind() {
    return executeAction(ElrsAction::Bind);
}

std::optional<Parameter> CrsfController::getParameter(uint8_t index) const {
    return impl_->parameters_.get(index);
}

void CrsfController::writeParameter(uint8_t index, const uint8_t* value, size_t len) {
    auto frame = frame_builder::buildParameterWrite(
        ADDR_CRSF_TRANSMITTER, ADDR_RADIO, index, value, len);
    impl_->serial_.writeCommand(frame);
}

bool CrsfController::areParametersLoaded() const {
    return impl_->parameters_.isReadComplete();
}

void CrsfController::ping() {
    auto frame = frame_builder::buildDevicePing(ADDR_BROADCAST, ADDR_RADIO);
    impl_->serial_.writeCommand(frame);
}

void CrsfController::setHeartbeatInterval(unsigned int microseconds) {
    impl_->heartbeat_interval_us_ = microseconds;
}

void CrsfController::setAutoReconnect(bool enabled, unsigned int interval_ms) {
    impl_->auto_reconnect_ = enabled;
    impl_->reconnect_interval_ms_ = interval_ms;
}

void CrsfController::onStateChange(StateCallback cb) {
    impl_->state_cb_ = std::move(cb);
}

void CrsfController::onParametersLoaded(VoidCallback cb) {
    impl_->params_loaded_cb_ = std::move(cb);
}

std::vector<std::string> CrsfController::listSerialPorts() {
    std::vector<std::string> ports;
#ifdef _WIN32
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 
                       0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        char valueName[256];
        char data[256];
        DWORD nameLen, dataLen, type;
        DWORD index = 0;
        
        while (true) {
            nameLen = sizeof(valueName);
            dataLen = sizeof(data);
            if (RegEnumValueA(hKey, index, valueName, &nameLen, nullptr, 
                              &type, reinterpret_cast<LPBYTE>(data), &dataLen) == ERROR_SUCCESS) {
                if (type == REG_SZ) {
                    ports.emplace_back(data);
                }
                index++;
            } else {
                break;
            }
        }
        RegCloseKey(hKey);
    }
#else
    // Linux: scan /dev/ttyUSB* and /dev/ttyACM*
    try {
        for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
            const std::string name = entry.path().filename().string();
            if (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
                ports.push_back(entry.path().string());
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[CRSF] Error scanning serial ports: " << e.what() << "\n";
    }
#endif
    return ports;
}

} // namespace crsf

#pragma once
#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>
#include <shared_mutex>
#include <mutex>

namespace mavlink_ctrl {

struct TelemetryData {
    // === HEARTBEAT (Msg 0) ===
    uint8_t remote_system_id = 0;
    uint8_t remote_component_id = 0;
    uint8_t remote_type = 0;             // MAV_TYPE
    uint8_t remote_autopilot = 0;        // MAV_AUTOPILOT
    uint8_t remote_base_mode = 0;
    uint32_t remote_custom_mode = 0;
    uint8_t remote_system_status = 0;    // MAV_STATE
    uint32_t last_heartbeat_ms = 0;

    // === SYS_STATUS (Msg 1) ===
    uint16_t battery_voltage_mv = 0;
    int16_t  battery_current_ca = 0;
    int8_t   battery_remaining = -1;
    uint16_t cpu_load = 0;
    uint16_t drop_rate = 0;
    uint16_t errors_comm = 0;

    // === GPS_RAW_INT (Msg 24) ===
    uint8_t  gps_fix_type = 0;
    int32_t  gps_lat = 0;
    int32_t  gps_lon = 0;
    int32_t  gps_alt_mm = 0;
    uint16_t gps_eph = 65535;
    uint16_t gps_epv = 65535;
    uint16_t gps_vel = 0;
    uint8_t  gps_satellites = 0;

    // === ATTITUDE (Msg 30) ===
    float attitude_roll = 0;
    float attitude_pitch = 0;
    float attitude_yaw = 0;
    float rollspeed = 0;
    float pitchspeed = 0;
    float yawspeed = 0;

    // === GLOBAL_POSITION_INT (Msg 33) ===
    int32_t global_lat = 0;
    int32_t global_lon = 0;
    int32_t global_alt_mm = 0;
    int32_t global_relative_alt_mm = 0;
    int16_t vx = 0;
    int16_t vy = 0;
    int16_t vz = 0;
    uint16_t hdg = 65535;

    // === SERVO_OUTPUT_RAW (Msg 36) ===
    uint16_t servo_raw[16] = {};

    // === RC_CHANNELS (Msg 65) ===
    uint16_t rc_channels[18] = {};
    uint8_t  rc_chancount = 0;
    uint8_t  rc_rssi = 0;

    // === VFR_HUD (Msg 74) ===
    float airspeed = 0;
    float groundspeed = 0;
    int16_t heading = 0;
    uint16_t throttle = 0;
    float alt = 0;
    float climb = 0;

    // === RADIO_STATUS (Msg 109) ===
    uint8_t radio_rssi = 0;
    uint8_t radio_remrssi = 0;
    uint8_t radio_txbuf = 0;
    uint8_t radio_noise = 0;
    uint8_t radio_remnoise = 0;
    uint16_t radio_rxerrors = 0;
    uint16_t radio_fixed = 0;

    // === STATUSTEXT (Msg 253) ===
    std::string last_statustext;
    uint8_t last_statustext_severity = 0;

    // === NAMED_VALUE_FLOAT (Msg 251) ===
    std::unordered_map<std::string, float> named_values;

    // === NAMED_VALUE_INT (Msg 252) ===
    std::unordered_map<std::string, int32_t> named_values_int;
};

class Telemetry {
public:
    TelemetryData get() const {
        std::shared_lock lock(mutex_);
        return data_;
    }

    void updateHeartbeat(uint8_t sys_id, uint8_t comp_id, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status, uint32_t last_heartbeat_ms) {
        std::unique_lock lock(mutex_);
        data_.remote_system_id = sys_id;
        data_.remote_component_id = comp_id;
        data_.remote_type = type;
        data_.remote_autopilot = autopilot;
        data_.remote_base_mode = base_mode;
        data_.remote_custom_mode = custom_mode;
        data_.remote_system_status = system_status;
        data_.last_heartbeat_ms = last_heartbeat_ms;
    }

    void updateSysStatus(uint16_t voltage_mv, int16_t current_ca, int8_t remaining, uint16_t cpu_load, uint16_t drop_rate, uint16_t errors_comm) {
        std::unique_lock lock(mutex_);
        data_.battery_voltage_mv = voltage_mv;
        data_.battery_current_ca = current_ca;
        data_.battery_remaining = remaining;
        data_.cpu_load = cpu_load;
        data_.drop_rate = drop_rate;
        data_.errors_comm = errors_comm;
    }

    void updateGpsRaw(uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt_mm, uint16_t eph, uint16_t epv, uint16_t vel, uint8_t satellites) {
        std::unique_lock lock(mutex_);
        data_.gps_fix_type = fix_type;
        data_.gps_lat = lat;
        data_.gps_lon = lon;
        data_.gps_alt_mm = alt_mm;
        data_.gps_eph = eph;
        data_.gps_epv = epv;
        data_.gps_vel = vel;
        data_.gps_satellites = satellites;
    }

    void updateAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
        std::unique_lock lock(mutex_);
        data_.attitude_roll = roll;
        data_.attitude_pitch = pitch;
        data_.attitude_yaw = yaw;
        data_.rollspeed = rollspeed;
        data_.pitchspeed = pitchspeed;
        data_.yawspeed = yawspeed;
    }

    void updateGlobalPosition(int32_t lat, int32_t lon, int32_t alt_mm, int32_t relative_alt_mm, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg) {
        std::unique_lock lock(mutex_);
        data_.global_lat = lat;
        data_.global_lon = lon;
        data_.global_alt_mm = alt_mm;
        data_.global_relative_alt_mm = relative_alt_mm;
        data_.vx = vx;
        data_.vy = vy;
        data_.vz = vz;
        data_.hdg = hdg;
    }

    void updateServoOutputRaw(const uint16_t servos[16]) {
        std::unique_lock lock(mutex_);
        for (int i = 0; i < 16; ++i) {
            data_.servo_raw[i] = servos[i];
        }
    }

    void updateRcChannels(const uint16_t channels[18], uint8_t chancount, uint8_t rssi) {
        std::unique_lock lock(mutex_);
        for (int i = 0; i < 18; ++i) {
            data_.rc_channels[i] = channels[i];
        }
        data_.rc_chancount = chancount;
        data_.rc_rssi = rssi;
    }

    void updateVfrHud(float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb) {
        std::unique_lock lock(mutex_);
        data_.airspeed = airspeed;
        data_.groundspeed = groundspeed;
        data_.heading = heading;
        data_.throttle = throttle;
        data_.alt = alt;
        data_.climb = climb;
    }

    void updateRadioStatus(uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed) {
        std::unique_lock lock(mutex_);
        data_.radio_rssi = rssi;
        data_.radio_remrssi = remrssi;
        data_.radio_txbuf = txbuf;
        data_.radio_noise = noise;
        data_.radio_remnoise = remnoise;
        data_.radio_rxerrors = rxerrors;
        data_.radio_fixed = fixed;
    }

    void updateStatustext(const std::string& text, uint8_t severity) {
        std::unique_lock lock(mutex_);
        data_.last_statustext = text;
        data_.last_statustext_severity = severity;
    }

    void updateNamedValueFloat(const std::string& name, float value) {
        std::unique_lock lock(mutex_);
        data_.named_values[name] = value;
    }

    void updateNamedValueInt(const std::string& name, int32_t value) {
        std::unique_lock lock(mutex_);
        data_.named_values_int[name] = value;
    }

    void reset() {
        std::unique_lock lock(mutex_);
        data_ = TelemetryData{};
    }

private:
    mutable std::shared_mutex mutex_;
    TelemetryData data_;
};

} // namespace mavlink_ctrl

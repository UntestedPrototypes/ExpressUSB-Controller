#pragma once
#include <cstdint>
#include <string>
#include <array>
#include <shared_mutex>
#include <mutex>

namespace crsf {

struct TelemetryData {
    // Link Statistics
    uint8_t uplink_rssi_1 = 0;
    uint8_t uplink_rssi_2 = 0;
    uint8_t uplink_lq = 0;          // 0-100%, 0 = no receiver
    int8_t  uplink_snr = 0;
    uint8_t active_antenna = 0;
    uint8_t rf_mode = 0;
    uint8_t uplink_tx_power = 0;
    uint8_t downlink_rssi = 0;
    uint8_t downlink_lq = 0;
    int8_t  downlink_snr = 0;

    // Battery
    uint16_t battery_voltage = 0;      // units: 100mV (126 = 12.6V)
    uint16_t battery_current = 0;      // units: 100mA
    uint32_t battery_capacity = 0;     // mAh consumed
    uint8_t  battery_remaining = 0;    // 0-100%

    // GPS
    int32_t  gps_latitude = 0;         // degree / 1e7
    int32_t  gps_longitude = 0;        // degree / 1e7
    uint16_t gps_groundspeed = 0;      // km/h / 100
    uint16_t gps_heading = 0;          // degree / 100
    uint16_t gps_altitude = 0;         // meters + 1000m offset
    uint8_t  gps_satellites = 0;

    // Attitude (radians * 10000)
    int16_t attitude_pitch = 0;
    int16_t attitude_roll = 0;
    int16_t attitude_yaw = 0;

    // Flight mode
    std::string flight_mode;

    // Device info
    std::string device_name;
    uint32_t serial_number = 0;
    uint32_t hardware_id = 0;
    uint32_t firmware_id = 0;
    uint8_t  parameters_count = 0;
    uint8_t  parameter_version = 0;

    // ELRS Extended (3.x and 4.x)
    // Vario (0x07)
    int16_t vario_verticalspd = 0; // cm/s

    // Baro Altitude (0x09)
    uint16_t baro_altitude = 0;    // meters + 1000m offset
    int16_t  baro_verticalspd = 0; // cm/s

    // Airspeed (0x0A)
    uint16_t airspeed_speed = 0;   // km/h / 10

    // GPS Time (0x03)
    uint16_t gps_time_year = 0;
    uint8_t  gps_time_month = 0;
    uint8_t  gps_time_day = 0;
    uint8_t  gps_time_hour = 0;
    uint8_t  gps_time_minute = 0;
    uint8_t  gps_time_second = 0;
    uint16_t gps_time_millisecond = 0;

    // ELRS Status (0x2E)
    uint8_t  elrs_pkts_bad = 0;
    uint16_t elrs_pkts_good = 0;
    uint8_t  elrs_flags = 0;
    std::string elrs_status_msg;

    // Handset Timing (0x3A)
    uint32_t handset_rate = 0;
    int32_t  handset_offset = 0;
    
    // Channels status byte (Arm state)
    bool channels_status_present = false;
    uint8_t channels_status = 0;
    bool is_armed = false;

    // Dynamic arrays
    uint8_t rpm_source_id = 0;
    std::vector<int32_t> rpm_values;

    uint8_t temp_source_id = 0;
    std::vector<int16_t> temp_values;

    uint8_t cells_source_id = 0;
    std::vector<uint16_t> cells_values; // Millivolt cell voltages
};

class Telemetry {
public:
    // Get a snapshot (copy) of all telemetry data
    TelemetryData get() const {
        std::shared_lock lock(mutex_);
        return data_;
    }

    // Update functions
    void updateLinkStatistics(uint8_t uplink_rssi_1, uint8_t uplink_rssi_2, uint8_t uplink_lq, int8_t uplink_snr, uint8_t active_antenna, uint8_t rf_mode, uint8_t uplink_tx_power, uint8_t downlink_rssi, uint8_t downlink_lq, int8_t downlink_snr) {
        std::unique_lock lock(mutex_);
        data_.uplink_rssi_1 = uplink_rssi_1;
        data_.uplink_rssi_2 = uplink_rssi_2;
        data_.uplink_lq = uplink_lq;
        data_.uplink_snr = uplink_snr;
        data_.active_antenna = active_antenna;
        data_.rf_mode = rf_mode;
        data_.uplink_tx_power = uplink_tx_power;
        data_.downlink_rssi = downlink_rssi;
        data_.downlink_lq = downlink_lq;
        data_.downlink_snr = downlink_snr;
    }

    void updateBattery(uint16_t voltage, uint16_t current, uint32_t capacity, uint8_t remaining) {
        std::unique_lock lock(mutex_);
        data_.battery_voltage = voltage;
        data_.battery_current = current;
        data_.battery_capacity = capacity;
        data_.battery_remaining = remaining;
    }

    void updateGps(int32_t lat, int32_t lon, uint16_t speed, uint16_t heading, uint16_t alt, uint8_t sats) {
        std::unique_lock lock(mutex_);
        data_.gps_latitude = lat;
        data_.gps_longitude = lon;
        data_.gps_groundspeed = speed;
        data_.gps_heading = heading;
        data_.gps_altitude = alt;
        data_.gps_satellites = sats;
    }

    void updateAttitude(int16_t pitch, int16_t roll, int16_t yaw) {
        std::unique_lock lock(mutex_);
        data_.attitude_pitch = pitch;
        data_.attitude_roll = roll;
        data_.attitude_yaw = yaw;
    }

    void updateFlightMode(const std::string& mode) {
        std::unique_lock lock(mutex_);
        data_.flight_mode = mode;
    }

    void updateDeviceInfo(const std::string& name, uint32_t serial, uint32_t hw_id, uint32_t fw_id, uint8_t param_count, uint8_t param_version) {
        std::unique_lock lock(mutex_);
        data_.device_name = name;
        data_.serial_number = serial;
        data_.hardware_id = hw_id;
        data_.firmware_id = fw_id;
        data_.parameters_count = param_count;
        data_.parameter_version = param_version;
    }
    
    // ELRS Extended Setters
    void updateVario(int16_t verticalspd) {
        std::unique_lock lock(mutex_);
        data_.vario_verticalspd = verticalspd;
    }

    void updateBaroAltitude(uint16_t altitude, int16_t verticalspd) {
        std::unique_lock lock(mutex_);
        data_.baro_altitude = altitude;
        data_.baro_verticalspd = verticalspd;
    }

    void updateAirspeed(uint16_t speed) {
        std::unique_lock lock(mutex_);
        data_.airspeed_speed = speed;
    }

    void updateGpsTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms) {
        std::unique_lock lock(mutex_);
        data_.gps_time_year = year;
        data_.gps_time_month = month;
        data_.gps_time_day = day;
        data_.gps_time_hour = hour;
        data_.gps_time_minute = min;
        data_.gps_time_second = sec;
        data_.gps_time_millisecond = ms;
    }

    void updateElrsStatus(uint8_t bad, uint16_t good, uint8_t flags, const std::string& msg) {
        std::unique_lock lock(mutex_);
        data_.elrs_pkts_bad = bad;
        data_.elrs_pkts_good = good;
        data_.elrs_flags = flags;
        data_.elrs_status_msg = msg;
    }

    void updateHandsetTiming(uint32_t rate, int32_t offset) {
        std::unique_lock lock(mutex_);
        data_.handset_rate = rate;
        data_.handset_offset = offset;
    }

    void updateChannelsStatus(uint8_t status) {
        std::unique_lock lock(mutex_);
        data_.channels_status_present = true;
        data_.channels_status = status;
        data_.is_armed = (status & 0x01) != 0; // CRSF_CHANNELS_STATUS_ARMED
    }

    void updateCells(uint8_t source_id, const std::vector<uint16_t>& cell_volts) {
        std::unique_lock lock(mutex_);
        data_.cells_source_id = source_id;
        data_.cells_values = cell_volts;
    }

    void updateTemp(uint8_t source_id, const std::vector<int16_t>& temps) {
        std::unique_lock lock(mutex_);
        data_.temp_source_id = source_id;
        data_.temp_values = temps;
    }

    void updateRpm(uint8_t source_id, const std::vector<int32_t>& rpms) {
        std::unique_lock lock(mutex_);
        data_.rpm_source_id = source_id;
        data_.rpm_values = rpms;
    }

    // Reset all to defaults
    void reset() {
        std::unique_lock lock(mutex_);
        data_ = TelemetryData{};
    }
    
private:
    mutable std::shared_mutex mutex_;
    TelemetryData data_;
};

} // namespace crsf

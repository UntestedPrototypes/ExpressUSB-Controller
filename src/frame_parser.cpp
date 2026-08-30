#include "crsf/frame_parser.h"
#include <iostream>

namespace crsf {

FrameParser::FrameParser(Telemetry& telemetry, ParameterStore& parameters)
    : telemetry_(telemetry), parameters_(parameters) {}

void FrameParser::parse(const std::vector<uint8_t>& frame) {
    if (frame.size() < 4) return;
    
    uint8_t type = frame[2];
    bool extended = (type >= 0x28);
    
    size_t payload_offset = extended ? 5 : 3;
    if (frame.size() <= payload_offset) return; // Malformed or empty frame beyond headers
    
    size_t payload_len = extended ? (frame[1] - 4) : (frame[1] - 2);
    
    if (frame.size() < payload_offset + payload_len + 1) {
        std::cerr << "[CRSF Parser] Frame size mismatch for type 0x" << std::hex << (int)type << std::dec << "\n";
        return; // Ensure CRC byte is there and payload matches
    }
    
    const uint8_t* payload = frame.data() + payload_offset;
    
    switch (type) {
        case 0x14: // Link Statistics
            handleLinkStatistics(payload, payload_len);
            break;
        case 0x08: // Battery Sensor
            handleBatterySensor(payload, payload_len);
            break;
        case 0x02: // GPS
            handleGPS(payload, payload_len);
            break;
        case 0x1E: // Attitude
            handleAttitude(payload, payload_len);
            break;
        case 0x21: // Flight Mode
            handleFlightMode(payload, payload_len);
            break;
        case 0x16: // RC Channels Packed
            handleRCChannelsPacked(payload, payload_len);
            break;
        case 0x29: // Device Info
            handleDeviceInfo(payload, payload_len);
            break;
        case 0x2B: // Parameter Settings Entry
            handleParameterSettingsEntry(payload, payload_len);
            break;
        case 0x10: // Radio ID / Ping? (placeholder)
            handleRadioId(payload, payload_len);
            break;
            
        // ELRS Extended (3.x and 4.x)
        case 0x07: // Vario
            handleVario(payload, payload_len);
            break;
        case 0x09: // Baro Altitude
            handleBaroAltitude(payload, payload_len);
            break;
        case 0x0A: // Airspeed
            handleAirspeed(payload, payload_len);
            break;
        case 0x03: // GPS Time
            handleGpsTime(payload, payload_len);
            break;
        case 0x2E: // ELRS Status
            handleElrsStatus(payload, payload_len);
            break;
        case 0x3A: // Handset Timing / Cmd
            handleHandsetTiming(payload, payload_len);
            break;
        case 0x0E: // Cells
            handleCells(payload, payload_len);
            break;
        case 0x0D: // Temp
            handleTemp(payload, payload_len);
            break;
        case 0x0C: // RPM
            handleRpm(payload, payload_len);
            break;
            
        default:
            // Unhandled type
            // std::cout << "[CRSF Parser] Unhandled frame type: 0x" << std::hex << (int)type << std::dec << "\n";
            break;
    }
}

void FrameParser::handleLinkStatistics(const uint8_t* payload, size_t len) {
    if (len < 10) return;
    telemetry_.updateLinkStatistics(
        payload[0], payload[1], payload[2], static_cast<int8_t>(payload[3]),
        payload[4], payload[5], payload[6], payload[7], payload[8], static_cast<int8_t>(payload[9])
    );
}

void FrameParser::handleBatterySensor(const uint8_t* payload, size_t len) {
    if (len < 8) return;
    uint16_t voltage = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    uint16_t current = (static_cast<uint16_t>(payload[2]) << 8) | payload[3];
    uint32_t capacity = (static_cast<uint32_t>(payload[4]) << 16) | (static_cast<uint32_t>(payload[5]) << 8) | payload[6]; // 24-bit? CRSF spec says 3 bytes capacity
    uint8_t remaining = payload[7];
    telemetry_.updateBattery(voltage, current, capacity, remaining);
}

void FrameParser::handleGPS(const uint8_t* payload, size_t len) {
    if (len < 15) return;
    int32_t lat = (static_cast<int32_t>(payload[0]) << 24) | (static_cast<int32_t>(payload[1]) << 16) | (static_cast<int32_t>(payload[2]) << 8) | payload[3];
    int32_t lon = (static_cast<int32_t>(payload[4]) << 24) | (static_cast<int32_t>(payload[5]) << 16) | (static_cast<int32_t>(payload[6]) << 8) | payload[7];
    uint16_t speed = (static_cast<uint16_t>(payload[8]) << 8) | payload[9];
    uint16_t heading = (static_cast<uint16_t>(payload[10]) << 8) | payload[11];
    uint16_t alt = (static_cast<uint16_t>(payload[12]) << 8) | payload[13];
    uint8_t sats = payload[14];
    telemetry_.updateGps(lat, lon, speed, heading, alt, sats);
}

void FrameParser::handleAttitude(const uint8_t* payload, size_t len) {
    if (len < 6) return;
    int16_t pitch = (static_cast<int16_t>(payload[0]) << 8) | payload[1];
    int16_t roll = (static_cast<int16_t>(payload[2]) << 8) | payload[3];
    int16_t yaw = (static_cast<int16_t>(payload[4]) << 8) | payload[5];
    telemetry_.updateAttitude(pitch, roll, yaw);
}

void FrameParser::handleFlightMode(const uint8_t* payload, size_t len) {
    std::string mode;
    for (size_t i = 0; i < len; ++i) {
        if (payload[i] == '\0') break;
        mode.push_back(static_cast<char>(payload[i]));
    }
    telemetry_.updateFlightMode(mode);
}

void FrameParser::handleRCChannelsPacked(const uint8_t* payload, size_t len) {
    if (len >= 23) {
        // ELRS 4.0+ arm state is packed into the 23rd byte (index 22)
        telemetry_.updateChannelsStatus(payload[22]);
    }
}

void FrameParser::handleDeviceInfo(const uint8_t* payload, size_t len) {
    std::string name;
    size_t offset = 0;
    while (offset < len && payload[offset] != '\0') {
        name.push_back(static_cast<char>(payload[offset]));
        offset++;
    }
    if (offset < len) offset++; // skip null terminator
    
    if (offset + 14 > len) {
        std::cerr << "[CRSF Parser] Device info payload too short\n";
        return;
    }
    
    uint32_t serial = (static_cast<uint32_t>(payload[offset]) << 24) | (static_cast<uint32_t>(payload[offset+1]) << 16) | (static_cast<uint32_t>(payload[offset+2]) << 8) | payload[offset+3];
    offset += 4;
    uint32_t hw_id = (static_cast<uint32_t>(payload[offset]) << 24) | (static_cast<uint32_t>(payload[offset+1]) << 16) | (static_cast<uint32_t>(payload[offset+2]) << 8) | payload[offset+3];
    offset += 4;
    uint32_t fw_id = (static_cast<uint32_t>(payload[offset]) << 24) | (static_cast<uint32_t>(payload[offset+1]) << 16) | (static_cast<uint32_t>(payload[offset+2]) << 8) | payload[offset+3];
    offset += 4;
    
    uint8_t param_count = payload[offset++];
    uint8_t param_version = payload[offset++];
    
    telemetry_.updateDeviceInfo(name, serial, hw_id, fw_id, param_count, param_version);
}

void FrameParser::handleParameterSettingsEntry(const uint8_t* payload, size_t len) {
    if (len < 2) return;
    
    uint8_t field_index = payload[0];
    uint8_t chunks_remaining = payload[1];
    
    parameters_.handleParameterEntry(field_index, chunks_remaining, payload + 2, len - 2);
}

void FrameParser::handleRadioId(const uint8_t* payload, size_t len) {
    // Unimplemented placeholder
}

// ELRS Extended Handlers
void FrameParser::handleVario(const uint8_t* payload, size_t len) {
    if (len < 2) return;
    int16_t verticalspd = (static_cast<int16_t>(payload[0]) << 8) | payload[1];
    telemetry_.updateVario(verticalspd);
}

void FrameParser::handleBaroAltitude(const uint8_t* payload, size_t len) {
    if (len < 4) return;
    uint16_t alt = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    int16_t vspd = (static_cast<int16_t>(payload[2]) << 8) | payload[3];
    telemetry_.updateBaroAltitude(alt, vspd);
}

void FrameParser::handleAirspeed(const uint8_t* payload, size_t len) {
    if (len < 2) return;
    uint16_t speed = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    telemetry_.updateAirspeed(speed);
}

void FrameParser::handleGpsTime(const uint8_t* payload, size_t len) {
    if (len < 9) return;
    uint16_t year = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    uint16_t ms = (static_cast<uint16_t>(payload[7]) << 8) | payload[8];
    telemetry_.updateGpsTime(year, payload[2], payload[3], payload[4], payload[5], payload[6], ms);
}

void FrameParser::handleElrsStatus(const uint8_t* payload, size_t len) {
    if (len < 4) return;
    uint8_t bad = payload[0];
    uint16_t good = (static_cast<uint16_t>(payload[1]) << 8) | payload[2];
    uint8_t flags = payload[3];
    
    std::string msg;
    for (size_t i = 4; i < len; ++i) {
        if (payload[i] == '\0') break;
        msg.push_back(static_cast<char>(payload[i]));
    }
    telemetry_.updateElrsStatus(bad, good, flags, msg);
}

void FrameParser::handleHandsetTiming(const uint8_t* payload, size_t len) {
    if (len < 9) return;
    if (payload[0] != 0x10) return; // CRSF_HANDSET_SUBCMD_TIMING
    uint32_t rate = (static_cast<uint32_t>(payload[1]) << 24) | (static_cast<uint32_t>(payload[2]) << 16) | (static_cast<uint32_t>(payload[3]) << 8) | payload[4];
    int32_t offset = (static_cast<int32_t>(payload[5]) << 24) | (static_cast<int32_t>(payload[6]) << 16) | (static_cast<int32_t>(payload[7]) << 8) | payload[8];
    telemetry_.updateHandsetTiming(rate, offset);
}

void FrameParser::handleCells(const uint8_t* payload, size_t len) {
    if (len < 1) return;
    uint8_t source_id = payload[0];
    std::vector<uint16_t> cells;
    for (size_t i = 1; i + 1 < len; i += 2) {
        cells.push_back((static_cast<uint16_t>(payload[i]) << 8) | payload[i+1]);
    }
    telemetry_.updateCells(source_id, cells);
}

void FrameParser::handleTemp(const uint8_t* payload, size_t len) {
    if (len < 1) return;
    uint8_t source_id = payload[0];
    std::vector<int16_t> temps;
    for (size_t i = 1; i + 1 < len; i += 2) {
        temps.push_back((static_cast<int16_t>(payload[i]) << 8) | payload[i+1]);
    }
    telemetry_.updateTemp(source_id, temps);
}

void FrameParser::handleRpm(const uint8_t* payload, size_t len) {
    if (len < 1) return;
    uint8_t source_id = payload[0];
    std::vector<int32_t> rpms;
    for (size_t i = 1; i + 2 < len; i += 3) {
        int32_t rpm = (static_cast<int32_t>(payload[i]) << 16) | (static_cast<int32_t>(payload[i+1]) << 8) | payload[i+2];
        if (rpm & 0x800000) rpm |= 0xFF000000; // Sign extend 24-bit to 32-bit
        rpms.push_back(rpm);
    }
    telemetry_.updateRpm(source_id, rpms);
}

} // namespace crsf

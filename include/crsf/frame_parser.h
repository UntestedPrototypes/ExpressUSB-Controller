#pragma once
#include <cstdint>
#include <vector>
#include <functional>
#include "crsf/crsf_protocol.h"
#include "crsf/telemetry.h"
#include "crsf/parameters.h"

namespace crsf {

class FrameParser {
public:
    FrameParser(Telemetry& telemetry, ParameterStore& parameters);
    
    // Parse a complete, CRC-validated frame
    void parse(const std::vector<uint8_t>& frame);
    
private:
    Telemetry& telemetry_;
    ParameterStore& parameters_;
    
    void handleLinkStatistics(const uint8_t* payload, size_t len);
    void handleBatterySensor(const uint8_t* payload, size_t len);
    void handleGPS(const uint8_t* payload, size_t len);
    void handleAttitude(const uint8_t* payload, size_t len);
    void handleFlightMode(const uint8_t* payload, size_t len);
    void handleRCChannelsPacked(const uint8_t* payload, size_t len);
    void handleDeviceInfo(const uint8_t* payload, size_t len);
    void handleParameterSettingsEntry(const uint8_t* payload, size_t len);
    void handleRadioId(const uint8_t* payload, size_t len);
    
    // ELRS Extended Handlers
    void handleVario(const uint8_t* payload, size_t len);
    void handleBaroAltitude(const uint8_t* payload, size_t len);
    void handleAirspeed(const uint8_t* payload, size_t len);
    void handleGpsTime(const uint8_t* payload, size_t len);
    void handleElrsStatus(const uint8_t* payload, size_t len);
    void handleHandsetTiming(const uint8_t* payload, size_t len);
    void handleCells(const uint8_t* payload, size_t len);
    void handleTemp(const uint8_t* payload, size_t len);
    void handleRpm(const uint8_t* payload, size_t len);
};

} // namespace crsf

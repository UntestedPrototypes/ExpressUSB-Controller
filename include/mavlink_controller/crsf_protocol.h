#pragma once
// CRSF (Crossfire / ExpressLRS) wire format helpers. Header-only, no dependencies.
//
// Frame:  [address][length][type][payload ...][crc8]
//   length : number of bytes after the length byte (type + payload + crc), 2..62
//   crc8   : CRC-8/DVB-S2 (polynomial 0xD5, initial value 0) over type + payload
//
// A PC acting as the "handset" sends RC_CHANNELS_PACKED frames (type 0x16, address 0xEE) to the TX
// module. The module answers with telemetry frames (link statistics, battery, GPS, attitude, flight mode ...).
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mavlink_ctrl {
namespace crsf {

// Frame addresses
constexpr uint8_t ADDR_FLIGHT_CONTROLLER = 0xC8;
constexpr uint8_t ADDR_RADIO_TRANSMITTER = 0xEA;  // what the TX module puts on frames it sends to the handset
constexpr uint8_t ADDR_CRSF_RECEIVER = 0xEC;
constexpr uint8_t ADDR_CRSF_TRANSMITTER = 0xEE;   // what a handset puts on frames it sends to the TX module

// Frame types
constexpr uint8_t TYPE_GPS = 0x02;
constexpr uint8_t TYPE_VARIO = 0x07;
constexpr uint8_t TYPE_BATTERY = 0x08;
constexpr uint8_t TYPE_LINK_STATISTICS = 0x14;
constexpr uint8_t TYPE_RC_CHANNELS = 0x16;
constexpr uint8_t TYPE_ATTITUDE = 0x1E;
constexpr uint8_t TYPE_FLIGHT_MODE = 0x21;

constexpr int NUM_CHANNELS = 16;
constexpr size_t MAX_FRAME_SIZE = 64;          // address + length + 62
constexpr size_t RC_PAYLOAD_SIZE = 22;         // 16 channels x 11 bit
constexpr uint16_t VALUE_MIN = 172;            // -100 %  (988 us)
constexpr uint16_t VALUE_MID = 992;            //    0 %  (1500 us)
constexpr uint16_t VALUE_MAX = 1811;           // +100 %  (2012 us)

inline uint8_t crc8(const uint8_t* data, size_t len, uint8_t crc = 0) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0xD5) : uint8_t(crc << 1);
    }
    return crc;
}

inline bool isKnownAddress(uint8_t b) {
    return b == ADDR_FLIGHT_CONTROLLER || b == ADDR_RADIO_TRANSMITTER || b == ADDR_CRSF_RECEIVER || b == ADDR_CRSF_TRANSMITTER;
}

// Microseconds (1500 = centre) <-> CRSF channel value (992 = centre), the scale radios and ELRS use.
inline uint16_t usToValue(uint16_t us) {
    int32_t v = (int32_t(us) - 1500) * 8 / 5 + VALUE_MID;
    if (v < VALUE_MIN) v = VALUE_MIN;
    if (v > VALUE_MAX) v = VALUE_MAX;
    return uint16_t(v);
}
inline uint16_t valueToUs(uint16_t v) {
    const int32_t d = int32_t(v) - VALUE_MID;
    return uint16_t(1500 + (d * 5 + (d >= 0 ? 4 : -4)) / 8);  // rounded to the nearest microsecond
}

inline std::vector<uint8_t> buildFrame(uint8_t address, uint8_t type, const uint8_t* payload, size_t payloadLen) {
    std::vector<uint8_t> f;
    f.reserve(payloadLen + 4);
    f.push_back(address);
    f.push_back(uint8_t(payloadLen + 2));
    f.push_back(type);
    f.insert(f.end(), payload, payload + payloadLen);
    f.push_back(crc8(f.data() + 2, payloadLen + 1));
    return f;
}

// 16 channels of 11 bit, least significant bit first
inline void packChannels(const uint16_t values[NUM_CHANNELS], uint8_t out[RC_PAYLOAD_SIZE]) {
    for (size_t i = 0; i < RC_PAYLOAD_SIZE; ++i) out[i] = 0;
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        const uint32_t v = values[ch] & 0x7FF;
        for (int bit = 0; bit < 11; ++bit) {
            if (v & (1u << bit)) {
                const int pos = ch * 11 + bit;
                out[pos >> 3] |= uint8_t(1u << (pos & 7));
            }
        }
    }
}

inline void unpackChannels(const uint8_t in[RC_PAYLOAD_SIZE], uint16_t values[NUM_CHANNELS]) {
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        uint32_t v = 0;
        for (int bit = 0; bit < 11; ++bit) {
            const int pos = ch * 11 + bit;
            if (in[pos >> 3] & (1u << (pos & 7))) v |= (1u << bit);
        }
        values[ch] = uint16_t(v);
    }
}

// The frame a handset sends to the TX module
inline std::vector<uint8_t> buildRcChannelsFrame(const uint16_t values[NUM_CHANNELS]) {
    uint8_t payload[RC_PAYLOAD_SIZE];
    packChannels(values, payload);
    return buildFrame(ADDR_CRSF_TRANSMITTER, TYPE_RC_CHANNELS, payload, RC_PAYLOAD_SIZE);
}

}  // namespace crsf
}  // namespace mavlink_ctrl

#pragma once

#include <cstdint>
#include <cstring>
#include <bit>

#if defined(_MSC_VER)
#define PACKED_STRUCT_BEGIN __pragma(pack(push, 1))
#define PACKED_STRUCT_END __pragma(pack(pop))
#define PACKED_ATTR
#else
#define PACKED_STRUCT_BEGIN
#define PACKED_STRUCT_END
#define PACKED_ATTR __attribute__((packed))
#endif

namespace crsf {

inline uint16_t crsf_bswap16(uint16_t val) {
    return static_cast<uint16_t>((val << 8) | (val >> 8));
}

inline uint32_t crsf_bswap32(uint32_t val) {
    return (val << 24) | ((val << 8) & 0x00FF0000) | ((val >> 8) & 0x0000FF00) | (val >> 24);
}

inline uint16_t crsf_htobe16(uint16_t val) {
    if constexpr (std::endian::native == std::endian::little) return crsf_bswap16(val);
    else return val;
}
inline uint16_t crsf_be16toh(uint16_t val) { return crsf_htobe16(val); }

inline uint32_t crsf_htobe32(uint32_t val) {
    if constexpr (std::endian::native == std::endian::little) return crsf_bswap32(val);
    else return val;
}
inline uint32_t crsf_be32toh(uint32_t val) { return crsf_htobe32(val); }

// Constants
constexpr uint8_t CRSF_CRC_POLY = 0xD5;
constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;
constexpr uint8_t CRSF_MAX_PACKET_LEN = 64;
constexpr uint8_t CRSF_PAYLOAD_SIZE_MAX = 62;
constexpr uint8_t CRSF_FRAME_NOT_COUNTED_BYTES = 2; // sync + length
constexpr uint8_t CRSF_CHANNELS_PAYLOAD_SIZE = 22;

// Channel constants
constexpr uint16_t CRSF_CHANNEL_MIN = 172;
constexpr uint16_t CRSF_CHANNEL_1000 = 191;
constexpr uint16_t CRSF_CHANNEL_MID = 992;
constexpr uint16_t CRSF_CHANNEL_2000 = 1792;
constexpr uint16_t CRSF_CHANNEL_MAX = 1811;

enum class FrameType : uint8_t {
    GPS = 0x02,
    VARIO = 0x07,
    BATTERY_SENSOR = 0x08,
    BARO_ALTITUDE = 0x09,
    HEARTBEAT = 0x0B,
    LINK_STATISTICS = 0x14,
    RC_CHANNELS_PACKED = 0x16,
    ATTITUDE = 0x1E,
    FLIGHT_MODE = 0x21,
    // Extended (>= 0x28):
    DEVICE_PING = 0x28,
    DEVICE_INFO = 0x29,
    PARAMETER_SETTINGS_ENTRY = 0x2B,
    PARAMETER_READ = 0x2C,
    PARAMETER_WRITE = 0x2D,
    COMMAND = 0x32,
    RADIO_ID = 0x3A
};

enum class Address : uint8_t {
    BROADCAST = 0x00,
    USB = 0x10,
    FLIGHT_CONTROLLER = 0xC8,
    RADIO_TRANSMITTER = 0xEA,
    CRSF_RECEIVER = 0xEC,
    CRSF_TRANSMITTER = 0xEE,
    ELRS_LUA = 0xEF
};

enum class ParameterType : uint8_t {
    UINT8 = 0,
    INT8 = 1,
    UINT16 = 2,
    INT16 = 3,
    UINT32 = 4,
    INT32 = 5,
    UINT64 = 6,
    INT64 = 7,
    FLOAT = 8,
    TEXT_SELECTION = 9,
    STRING = 10,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
    VTX = 15,
    OUT_OF_RANGE = 127
};

constexpr uint8_t CRSF_FIELD_HIDDEN = 0x80;
constexpr uint8_t CRSF_FIELD_ELRS_HIDDEN = 0x40;
constexpr uint8_t CRSF_FIELD_TYPE_MASK = 0x3F;

PACKED_STRUCT_BEGIN
struct crsf_header_t {
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_ext_header_t {
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_channels_t {
    // 16 channels, 11-bit each, total 22 bytes. 
    // We declare an array of 22 bytes instead of bitfields due to portability.
    uint8_t data[22];
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_sensor_battery_t {
    uint16_t voltage; // Big Endian
    uint16_t current; // Big Endian
    uint8_t capacity[3]; // 24-bit Big Endian
    uint8_t remaining;
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_sensor_gps_t {
    int32_t latitude;  // Big Endian
    int32_t longitude; // Big Endian
    uint16_t groundspeed; // Big Endian
    uint16_t heading; // Big Endian
    uint16_t altitude; // Big Endian
    uint8_t satellites;
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_sensor_attitude_t {
    int16_t pitch; // Big Endian
    int16_t roll; // Big Endian
    int16_t yaw; // Big Endian
} PACKED_ATTR;
PACKED_STRUCT_END

PACKED_STRUCT_BEGIN
struct crsf_link_statistics_t {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;
    int8_t uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t downlink_snr;
} PACKED_ATTR;
PACKED_STRUCT_END

// Integer-based proportional mapping (avoids floating-point imprecision)
inline uint16_t fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return static_cast<uint16_t>(
        ((x - in_min) * (out_max - out_min) * 2 / (in_max - in_min) + out_min * 2 + 1) / 2
    );
}

// CRSF raw (172-1811) → PWM microseconds (988-2012)
inline uint16_t crsfToUs(uint16_t val) {
    return fmap(val, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, 988, 2012);
}

// PWM microseconds (988-2012) → CRSF raw (172-1811)
inline uint16_t usToCrsf(uint16_t val) {
    return fmap(val, 988, 2012, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
}

// 10-bit (0-1023) → CRSF raw
inline uint16_t uint10ToCrsf(uint16_t val) {
    return fmap(val, 0, 1023, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
}

// CRSF raw → 10-bit (0-1023)
inline uint16_t crsfToUint10(uint16_t val) {
    return fmap(val, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, 0, 1023);
}

inline bool isExtendedFrameType(uint8_t type) {
    return type >= 0x28;
}

} // namespace crsf

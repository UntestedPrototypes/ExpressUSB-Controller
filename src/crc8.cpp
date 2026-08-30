#include "crsf/crc8.h"

namespace crsf {

CRC8::CRC8(uint8_t polynomial) {
    for (int i = 0; i < 256; ++i) {
        uint8_t crc = static_cast<uint8_t>(i);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ polynomial);
            } else {
                crc <<= 1;
            }
        }
        table_[i] = crc;
    }
}

uint8_t CRC8::calc(uint8_t data) const {
    return table_[data];
}

uint8_t CRC8::calc(const uint8_t* data, size_t len, uint8_t crc) const {
    for (size_t i = 0; i < len; ++i) {
        crc = table_[crc ^ data[i]];
    }
    return crc;
}

} // namespace crsf

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace crsf {

class CRC8 {
public:
    explicit CRC8(uint8_t polynomial = 0xD5);
    uint8_t calc(uint8_t data) const;
    uint8_t calc(const uint8_t* data, size_t len, uint8_t crc = 0) const;

private:
    std::array<uint8_t, 256> table_{};
};

} // namespace crsf

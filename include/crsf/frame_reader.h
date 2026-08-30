#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <set>
#include <initializer_list>
#include "crsf/crc8.h"

namespace crsf {

class FrameReader {
public:
    using FrameCallback = std::function<void(const std::vector<uint8_t>&)>;
    
    // Constructor takes list of valid sync bytes to accept
    explicit FrameReader(std::initializer_list<uint8_t> sync_addresses);
    
    // Set callback for complete, validated frames
    void setFrameCallback(FrameCallback cb);
    
    // Feed bytes from serial port
    void feed(uint8_t byte);
    void feed(const uint8_t* data, size_t len);
    
    // Reset state machine
    void reset();
    
    // Statistics
    struct Stats {
        uint32_t frames_received = 0;
        uint32_t crc_errors = 0;
        uint32_t length_errors = 0;
    };
    Stats getStats() const;
    
private:
    enum class State { WAIT_SYNC, READ_LENGTH, READ_PAYLOAD };
    
    State state_ = State::WAIT_SYNC;
    std::vector<uint8_t> frame_buffer_;
    uint8_t expected_length_ = 0;
    std::set<uint8_t> sync_addresses_;
    FrameCallback frame_callback_;
    CRC8 crc_;
    Stats stats_;
};

} // namespace crsf

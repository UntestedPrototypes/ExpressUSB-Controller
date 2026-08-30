#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>

namespace crsf::frame_builder {

    // Build a standard broadcast frame
    std::vector<uint8_t> buildFrame(uint8_t dest_addr, uint8_t frame_type, 
                                    const uint8_t* payload, size_t payload_len);
    
    // Build an extended header frame  
    std::vector<uint8_t> buildExtFrame(uint8_t sync_addr, uint8_t frame_type,
                                       uint8_t dest_addr, uint8_t orig_addr,
                                       const uint8_t* payload, size_t payload_len);
    
    // Build RC channels frame (16 channels, 11-bit each, packed into 22 bytes)
    // channels array must have 16 elements, values 0-2047 (CRSF raw)
    std::vector<uint8_t> buildRCChannelsFrame(const uint16_t channels[16]);
    
    // Build device ping (extended frame: sync=0xC8, type=0x28, payload=empty)
    std::vector<uint8_t> buildDevicePing(uint8_t dest_addr, uint8_t orig_addr);
    
    // Build parameter read request
    std::vector<uint8_t> buildParameterRead(uint8_t dest_addr, uint8_t orig_addr,
                                            uint8_t param_index, uint8_t chunk_index);
    
    // Build parameter write
    std::vector<uint8_t> buildParameterWrite(uint8_t dest_addr, uint8_t orig_addr,
                                             uint8_t param_index, 
                                             const uint8_t* value, size_t value_len);

} // namespace crsf::frame_builder

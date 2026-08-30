#include "crsf/frame_builder.h"
#include "crsf/crsf_protocol.h"
#include "crsf/crc8.h"

namespace crsf::frame_builder {

std::vector<uint8_t> buildFrame(uint8_t dest_addr, uint8_t frame_type, 
                                const uint8_t* payload, size_t payload_len) {
    std::vector<uint8_t> frame;
    frame.reserve(1 + 1 + 1 + payload_len + 1); // sync + len + type + payload + crc

    frame.push_back(dest_addr);
    frame.push_back(static_cast<uint8_t>(1 + payload_len + 1)); // type + payload + crc
    frame.push_back(frame_type);

    if (payload && payload_len > 0) {
        frame.insert(frame.end(), payload, payload + payload_len);
    }

    CRC8 crc_calc;
    uint8_t crc = crc_calc.calc(&frame[2], frame.size() - 2); // over type + payload
    frame.push_back(crc);

    return frame;
}

std::vector<uint8_t> buildExtFrame(uint8_t sync_addr, uint8_t frame_type,
                                   uint8_t dest_addr, uint8_t orig_addr,
                                   const uint8_t* payload, size_t payload_len) {
    std::vector<uint8_t> frame;
    frame.reserve(1 + 1 + 1 + 1 + 1 + payload_len + 1); // sync + len + type + dest + orig + payload + crc

    frame.push_back(sync_addr);
    frame.push_back(static_cast<uint8_t>(1 + 1 + 1 + payload_len + 1)); // type + dest + orig + payload + crc
    frame.push_back(frame_type);
    frame.push_back(dest_addr);
    frame.push_back(orig_addr);

    if (payload && payload_len > 0) {
        frame.insert(frame.end(), payload, payload + payload_len);
    }

    CRC8 crc_calc;
    uint8_t crc = crc_calc.calc(&frame[2], frame.size() - 2); // over type + dest + orig + payload
    frame.push_back(crc);

    return frame;
}

std::vector<uint8_t> buildRCChannelsFrame(const uint16_t channels[16]) {
    uint8_t payload[22] = {0};

    payload[0]  = static_cast<uint8_t>(channels[0]);
    payload[1]  = static_cast<uint8_t>((channels[0] >> 8) | (channels[1] << 3));
    payload[2]  = static_cast<uint8_t>((channels[1] >> 5) | (channels[2] << 6));
    payload[3]  = static_cast<uint8_t>(channels[2] >> 2);
    payload[4]  = static_cast<uint8_t>((channels[2] >> 10) | (channels[3] << 1));
    payload[5]  = static_cast<uint8_t>((channels[3] >> 7) | (channels[4] << 4));
    payload[6]  = static_cast<uint8_t>((channels[4] >> 4) | (channels[5] << 7));
    payload[7]  = static_cast<uint8_t>(channels[5] >> 1);
    payload[8]  = static_cast<uint8_t>((channels[5] >> 9) | (channels[6] << 2));
    payload[9]  = static_cast<uint8_t>((channels[6] >> 6) | (channels[7] << 5));
    payload[10] = static_cast<uint8_t>(channels[7] >> 3);
    payload[11] = static_cast<uint8_t>(channels[8]);
    payload[12] = static_cast<uint8_t>((channels[8] >> 8) | (channels[9] << 3));
    payload[13] = static_cast<uint8_t>((channels[9] >> 5) | (channels[10] << 6));
    payload[14] = static_cast<uint8_t>(channels[10] >> 2);
    payload[15] = static_cast<uint8_t>((channels[10] >> 10) | (channels[11] << 1));
    payload[16] = static_cast<uint8_t>((channels[11] >> 7) | (channels[12] << 4));
    payload[17] = static_cast<uint8_t>((channels[12] >> 4) | (channels[13] << 7));
    payload[18] = static_cast<uint8_t>(channels[13] >> 1);
    payload[19] = static_cast<uint8_t>((channels[13] >> 9) | (channels[14] << 2));
    payload[20] = static_cast<uint8_t>((channels[14] >> 6) | (channels[15] << 5));
    payload[21] = static_cast<uint8_t>(channels[15] >> 3);

    return buildFrame(static_cast<uint8_t>(Address::CRSF_TRANSMITTER), static_cast<uint8_t>(FrameType::RC_CHANNELS_PACKED), payload, 22);
}

std::vector<uint8_t> buildDevicePing(uint8_t dest_addr, uint8_t orig_addr) {
    return buildExtFrame(CRSF_SYNC_BYTE, static_cast<uint8_t>(FrameType::DEVICE_PING), dest_addr, orig_addr, nullptr, 0);
}

std::vector<uint8_t> buildParameterRead(uint8_t dest_addr, uint8_t orig_addr, uint8_t param_index, uint8_t chunk_index) {
    uint8_t payload[2] = {param_index, chunk_index};
    return buildExtFrame(CRSF_SYNC_BYTE, static_cast<uint8_t>(FrameType::PARAMETER_READ), dest_addr, orig_addr, payload, 2);
}

std::vector<uint8_t> buildParameterWrite(uint8_t dest_addr, uint8_t orig_addr, uint8_t param_index, const uint8_t* value, size_t value_len) {
    std::vector<uint8_t> payload;
    payload.reserve(1 + value_len);
    payload.push_back(param_index);
    if (value && value_len > 0) {
        payload.insert(payload.end(), value, value + value_len);
    }
    return buildExtFrame(CRSF_SYNC_BYTE, static_cast<uint8_t>(FrameType::PARAMETER_WRITE), dest_addr, orig_addr, payload.data(), payload.size());
}

} // namespace crsf::frame_builder

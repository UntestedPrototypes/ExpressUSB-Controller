#include "crsf/frame_reader.h"

namespace crsf {

FrameReader::FrameReader(std::initializer_list<uint8_t> sync_addresses)
    : sync_addresses_(sync_addresses), crc_(0xD5) {
}

void FrameReader::setFrameCallback(FrameCallback cb) {
    frame_callback_ = std::move(cb);
}

void FrameReader::feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        feed(data[i]);
    }
}

void FrameReader::feed(uint8_t byte) {
    switch (state_) {
        case State::WAIT_SYNC:
            if (sync_addresses_.count(byte)) {
                frame_buffer_.clear();
                frame_buffer_.push_back(byte);
                state_ = State::READ_LENGTH;
            }
            break;
            
        case State::READ_LENGTH:
            if (byte < 2 || byte > 62) {
                stats_.length_errors++;
                state_ = State::WAIT_SYNC;
            } else {
                expected_length_ = byte;
                frame_buffer_.push_back(byte);
                frame_buffer_.reserve(expected_length_ + 2);
                state_ = State::READ_PAYLOAD;
            }
            break;
            
        case State::READ_PAYLOAD:
            frame_buffer_.push_back(byte);
            
            if (frame_buffer_.size() == static_cast<size_t>(expected_length_ + 2)) {
                // We have the whole frame. Validate CRC.
                uint8_t calculated_crc = crc_.calc(&frame_buffer_[2], frame_buffer_.size() - 3);
                uint8_t received_crc = frame_buffer_.back();
                
                if (calculated_crc == received_crc) {
                    stats_.frames_received++;
                    if (frame_callback_) {
                        frame_callback_(frame_buffer_);
                    }
                } else {
                    stats_.crc_errors++;
                }
                
                // Done with this frame
                state_ = State::WAIT_SYNC;
            }
            break;
    }
}

void FrameReader::reset() {
    state_ = State::WAIT_SYNC;
    frame_buffer_.clear();
    expected_length_ = 0;
}

FrameReader::Stats FrameReader::getStats() const {
    return stats_;
}

} // namespace crsf

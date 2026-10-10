#pragma once
// CRSF link: the PC acts as the handset of an ELRS TX module that runs in CRSF mode.
//   up   : RC_CHANNELS_PACKED frames (16 channels)
//   down : link statistics, battery, GPS, attitude, flight mode (text) frames from the TX module
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "mavlink_controller/crsf_protocol.h"
#include "mavlink_controller/link_protocol.h"
#include "mavlink_controller/telemetry.h"

namespace mavlink_ctrl {

class CrsfHandler : public LinkProtocol {
public:
    using FrameCallback = std::function<void(uint8_t type, const uint8_t* payload, size_t length)>;

    explicit CrsfHandler(Telemetry& telemetry);

    // Called for every valid frame, whatever its type (a tap for frames this class does not decode)
    void setFrameCallback(FrameCallback cb) { frame_cb_ = std::move(cb); }

    // How long link statistics with a link quality above zero keep the link "up"
    void setLinkTimeout(uint32_t ms) { link_timeout_ms_ = ms; }

    // LinkProtocol
    LinkMode mode() const override { return LinkMode::Crsf; }
    void processBytes(const uint8_t* data, size_t len) override;
    std::vector<uint8_t> buildKeepAlive() override { return {}; }  // CRSF needs no heartbeat
    // 0 and UINT16_MAX = no value: the channel keeps what it had (centred before the first value).
    // Only the first 16 of the 18 channels exist in CRSF.
    std::vector<uint8_t> buildRcFrame(const uint16_t channels[18]) override;
    bool linkUp() const override;
    void reset() override;

    uint32_t framesOk() const { return frames_ok_; }
    uint32_t crcErrors() const { return crc_errors_; }

private:
    void feed(uint8_t b);
    void handleFrame(uint8_t type, const uint8_t* payload, size_t len);

    Telemetry& telemetry_;
    FrameCallback frame_cb_;
    uint32_t link_timeout_ms_ = 3000;

    // receive buffer: always starts at a candidate frame start
    uint8_t buf_[crsf::MAX_FRAME_SIZE] = {};
    size_t fill_ = 0;
    std::atomic<uint32_t> frames_ok_{0}, crc_errors_{0};
    std::atomic<uint32_t> last_link_ms_{0};
    std::atomic<bool> link_seen_{false};

    // sticky channel values (CRSF scale); the RC thread and reset() both touch them
    std::mutex values_mutex_;
    uint16_t last_values_[crsf::NUM_CHANNELS];
};

}  // namespace mavlink_ctrl

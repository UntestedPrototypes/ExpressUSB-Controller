#include "mavlink_controller/crsf_handler.h"

#include <chrono>
#include <cstring>
#include <string>

namespace mavlink_ctrl {

namespace {

uint32_t nowMs() {
    return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}

// CRSF telemetry numbers are big-endian
int16_t be16s(const uint8_t* p) { return int16_t((p[0] << 8) | p[1]); }
uint16_t be16u(const uint8_t* p) { return uint16_t((p[0] << 8) | p[1]); }
int32_t be32s(const uint8_t* p) {
    return int32_t((uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3]);
}

}  // namespace

CrsfHandler::CrsfHandler(Telemetry& telemetry) : telemetry_(telemetry) {
    for (auto& v : last_values_) v = crsf::VALUE_MID;
}

void CrsfHandler::reset() {
    fill_ = 0;
    link_seen_ = false;
    last_link_ms_ = 0;
    frames_ok_ = 0;
    crc_errors_ = 0;
    std::lock_guard<std::mutex> lock(values_mutex_);
    for (auto& v : last_values_) v = crsf::VALUE_MID;
}

bool CrsfHandler::linkUp() const {
    return link_seen_ && nowMs() - last_link_ms_ <= link_timeout_ms_;
}

void CrsfHandler::processBytes(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) feed(data[i]);
}

// Stream scanner with resynchronisation: after a bad checksum or a wrong length the first byte of
// the buffer is dropped and the rest is scanned again, so one corrupted byte costs one frame at most.
void CrsfHandler::feed(uint8_t b) {
    if (fill_ >= sizeof(buf_)) fill_ = 0;  // cannot happen with the length checks below; defensive
    buf_[fill_++] = b;

    for (;;) {
        // find a plausible frame start
        size_t skip = 0;
        while (skip < fill_ && !crsf::isKnownAddress(buf_[skip])) ++skip;
        if (skip) {
            std::memmove(buf_, buf_ + skip, fill_ - skip);
            fill_ -= skip;
        }
        if (fill_ < 2) return;

        const size_t length = buf_[1];  // type + payload + crc
        if (length < 2 || length > 62) {  // not a frame: drop the address byte and look again
            std::memmove(buf_, buf_ + 1, fill_ - 1);
            --fill_;
            continue;
        }
        const size_t total = length + 2;
        if (fill_ < total) return;  // wait for the rest

        if (crsf::crc8(buf_ + 2, length - 1) == buf_[total - 1]) {
            ++frames_ok_;
            handleFrame(buf_[2], buf_ + 3, length - 2);
            std::memmove(buf_, buf_ + total, fill_ - total);
            fill_ -= total;
        } else {
            ++crc_errors_;
            std::memmove(buf_, buf_ + 1, fill_ - 1);
            --fill_;
        }
        telemetry_.updateCrsfCounters(frames_ok_, crc_errors_);
    }
}

void CrsfHandler::handleFrame(uint8_t type, const uint8_t* p, size_t n) {
    switch (type) {
        case crsf::TYPE_LINK_STATISTICS:
            if (n >= 10) {
                Telemetry::CrsfLinkStats s{p[0], p[1], p[2], int8_t(p[3]), p[4], p[5], p[6], p[7], p[8], int8_t(p[9])};
                const uint32_t now = nowMs();
                telemetry_.updateCrsfLinkStats(s, now);
                if (s.uplink_lq > 0) {
                    last_link_ms_ = now;
                    link_seen_ = true;
                } else {
                    link_seen_ = false;  // the module reports no link
                }
            }
            break;

        case crsf::TYPE_BATTERY:  // voltage 0.1 V, current 0.1 A (int16), capacity mAh (24 bit), remaining %
            if (n >= 8) {
                const int32_t mv = int32_t(be16s(p)) * 100;
                const int32_t ca = int32_t(be16s(p + 2)) * 10;
                telemetry_.updateCrsfBattery(uint16_t(mv < 0 ? 0 : mv), int16_t(ca), int8_t(p[7]));
            }
            break;

        case crsf::TYPE_ATTITUDE:  // pitch, roll, yaw in 1e-4 rad
            if (n >= 6) {
                telemetry_.updateAttitude(float(be16s(p + 2)) / 1e4f, float(be16s(p)) / 1e4f, float(be16s(p + 4)) / 1e4f, 0, 0, 0);
            }
            break;

        case crsf::TYPE_GPS:  // lat/lon 1e-7 deg, speed 0.1 km/h, heading 0.01 deg, altitude m + 1000, satellites
            if (n >= 15) {
                const int32_t lat = be32s(p), lon = be32s(p + 4);
                const uint16_t speed = be16u(p + 8);  // 0.1 km/h -> cm/s
                const uint16_t sats = p[14];
                const int32_t alt_mm = (int32_t(be16u(p + 12)) - 1000) * 1000;
                telemetry_.updateGpsRaw(sats > 0 ? 3 : 0, lat, lon, alt_mm, 65535, 65535, uint16_t(uint32_t(speed) * 100000u / 36000u),
                                        uint8_t(sats));
            }
            break;

        case crsf::TYPE_FLIGHT_MODE: {  // zero-terminated text
            size_t len = 0;
            while (len < n && p[len] != 0) ++len;
            telemetry_.updateFlightMode(std::string(reinterpret_cast<const char*>(p), len));
            break;
        }

        default:
            break;
    }
    if (frame_cb_) frame_cb_(type, p, n);
}

std::vector<uint8_t> CrsfHandler::buildRcFrame(const uint16_t channels[18]) {
    uint16_t values[crsf::NUM_CHANNELS];
    std::lock_guard<std::mutex> lock(values_mutex_);
    for (int i = 0; i < crsf::NUM_CHANNELS; ++i) {
        const uint16_t us = channels[i];
        if (us != 0 && us != UINT16_MAX) last_values_[i] = crsf::usToValue(us);
        values[i] = last_values_[i];
    }
    return crsf::buildRcChannelsFrame(values);
}

}  // namespace mavlink_ctrl

#pragma once
// What the controller needs from a radio link protocol. MavlinkHandler (native MAVLink mode) and
// CrsfHandler (CRSF mode) both implement it; MavlinkController drives whichever one was selected.
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mavlink_ctrl {

enum class LinkMode {
    Mavlink,  // ELRS "Native MAVLink": RC_CHANNELS_OVERRIDE up, MAVLink telemetry down
    Crsf      // ELRS with the PC acting as the CRSF handset: RC channels frame up, CRSF telemetry down
};

class LinkProtocol {
public:
    virtual ~LinkProtocol() = default;

    virtual LinkMode mode() const = 0;

    // Bytes received from the serial port
    virtual void processBytes(const uint8_t* data, size_t len) = 0;

    // Frame that has to be sent periodically to keep the link alive (MAVLink: GCS heartbeat).
    // Empty if the protocol needs none.
    virtual std::vector<uint8_t> buildKeepAlive() = 0;

    // Frame carrying the RC channels. channels: 18 values in microseconds; 0 and UINT16_MAX mean
    // "no value" (see the protocol's documentation for what that does).
    virtual std::vector<uint8_t> buildRcFrame(const uint16_t channels[18]) = 0;

    // True while the remote end is demonstrably alive (MAVLink: heartbeat seen; CRSF: link
    // statistics with a link quality above zero seen recently).
    virtual bool linkUp() const = 0;

    // Forget everything learned from the previous connection
    virtual void reset() = 0;
};

}  // namespace mavlink_ctrl

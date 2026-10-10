#pragma once
// A byte pipe to the TX module. SerialPort is the real one; the self-tests use a loopback.
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mavlink_ctrl {

class ITransport {
public:
    using BytesCallback = std::function<void(const uint8_t*, size_t)>;

    virtual ~ITransport() = default;

    virtual bool open(const std::string& port_name, unsigned int baud_rate) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual void setReceiveCallback(BytesCallback cb) = 0;

    // RC frames replace a pending RC frame that was not written yet; commands queue FIFO
    virtual void writeRC(const std::vector<uint8_t>& data) = 0;
    virtual void writeCommand(const std::vector<uint8_t>& data) = 0;
};

}  // namespace mavlink_ctrl

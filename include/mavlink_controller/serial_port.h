#pragma once
#include <asio.hpp>
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <condition_variable>

#include "mavlink_controller/transport.h"

namespace mavlink_ctrl {

class SerialPort : public ITransport {
public:
    SerialPort();
    ~SerialPort() override;

    // Open serial port. Returns true on success. Rates that are not standard POSIX rates (CRSF uses
    // 400000) are set through termios2 on Linux; Windows takes any rate.
    bool open(const std::string& port_name, unsigned int baud_rate) override;

    // Close and clean up everything
    void close() override;

    // Check if port is open and operational
    bool isOpen() const override;

    // Set callback for received bytes
    void setReceiveCallback(BytesCallback cb) override;

    // Write data with priority: RC frames replace pending RC, commands queue FIFO
    void writeRC(const std::vector<uint8_t>& data) override;
    void writeCommand(const std::vector<uint8_t>& data) override;
    
private:
    std::unique_ptr<asio::io_context> io_context_;
    std::unique_ptr<asio::serial_port> serial_;
    std::vector<std::thread> io_threads_;
    std::atomic<bool> is_open_{false};
    
    BytesCallback receive_callback_;
    
    // Read
    static constexpr size_t READ_BUF_SIZE = 256;
    std::array<uint8_t, READ_BUF_SIZE> read_buffer_{};
    void readThreadLoop();
    
    // Write queue management
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    std::vector<uint8_t> pending_rc_frame_;
    bool rc_pending_ = false;
    std::deque<std::vector<uint8_t>> command_queue_;
    
    void writeThreadLoop();
};

} // namespace mavlink_ctrl

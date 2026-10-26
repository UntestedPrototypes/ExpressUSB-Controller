#pragma once
#include <asio.hpp>
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <array>
#include <deque>

namespace crsf {

class SerialPort {
public:
    using BytesCallback = std::function<void(const uint8_t*, size_t)>;
    
    SerialPort();
    ~SerialPort();
    
    // Open serial port. Returns true on success.
    bool open(const std::string& port_name, unsigned int baud_rate);
    
    // Close and clean up everything
    void close();
    
    // Check if port is open and operational
    bool isOpen() const;
    
    // Set callback for received bytes
    void setReceiveCallback(BytesCallback cb);
    
    // Write data with priority: RC frames replace pending RC, commands queue FIFO
    void writeRC(const std::vector<uint8_t>& data);
    void writeCommand(const std::vector<uint8_t>& data);

    // Reboot an ESP32 behind a USB-serial bridge with the usual auto-reset wiring (RTS -> EN, DTR -> IO0), the
    // same pulse esptool uses: DTR off (normal boot, not the bootloader), RTS on (EN low) for hold_ms, RTS off.
    // Opens and closes the port itself, so the port must not be open elsewhere. False if it can't be opened.
    // Does nothing on boards whose reset is not wired to these lines.
    static bool pulseReset(const std::string& port_name, unsigned int hold_ms = 100);

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
    void startRead();
    
    // Write queue management
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    std::vector<uint8_t> pending_rc_frame_;
    bool rc_pending_ = false;
    std::deque<std::vector<uint8_t>> command_queue_;
    bool write_in_progress_ = false;
    std::vector<uint8_t> active_write_buffer_; // keeps data alive during async_write
    
    void writeThreadLoop();
    void doNextWrite();
};

} // namespace crsf

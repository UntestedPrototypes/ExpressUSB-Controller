#include "crsf/serial_port.h"
#include <iostream>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace crsf {

bool SerialPort::pulseReset(const std::string& port_name, unsigned int hold_ms) {
    // Never DTR on with RTS off: on the auto-reset circuit that pulls IO0 low, which would start the bootloader.
#ifdef _WIN32
    HANDLE h = CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, SETRTS);   // EN low: held in reset
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    EscapeCommFunction(h, CLRRTS);   // EN high, IO0 high: normal boot
    CloseHandle(h);
#else
    const int fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;
    int dtr = TIOCM_DTR, rts = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &dtr);
    ioctl(fd, TIOCMBIS, &rts);
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    ioctl(fd, TIOCMBIC, &rts);
    ::close(fd);
#endif
    return true;
}

SerialPort::SerialPort() {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port_name, unsigned int baud_rate) {
    if (is_open_) close();
    
    io_context_ = std::make_unique<asio::io_context>();
    serial_ = std::make_unique<asio::serial_port>(*io_context_);
    
    try {
        serial_->open(port_name);
        
        serial_->set_option(asio::serial_port_base::baud_rate(baud_rate));
        serial_->set_option(asio::serial_port_base::character_size(8));
        serial_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial_->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    } catch (const std::exception& e) {
        std::cerr << "[Serial] Port open error: " << e.what() << "\n";
        return false;
    }

    is_open_ = true;

    // Thread 0: Dedicated Manual Read Thread
    io_threads_.emplace_back([this]() {
        readThreadLoop();
    });

    // Thread 1: Dedicated Manual Write Thread
    io_threads_.emplace_back([this]() {
        writeThreadLoop();
    });

    return true;
}

void SerialPort::close() {
    if (!is_open_) return;
    is_open_ = false;  // Prevent new reads/writes from being queued

    // Wake up the write thread so it can exit
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_cv_.notify_all();
    }

    if (io_context_ && serial_) {
        // cancel + close on the io thread; its pending read then completes with operation_aborted and run() returns
        asio::post(*io_context_, [this]() {
            std::error_code ec;
            if (serial_ && serial_->is_open()) { serial_->cancel(ec); serial_->close(ec); }
        });
    }

    for (auto& t : io_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
    io_threads_.clear();

    serial_.reset();
    io_context_.reset();
    
    std::lock_guard<std::mutex> lock(write_mutex_);
    rc_pending_ = false;
    write_in_progress_ = false;
    command_queue_.clear();
}

bool SerialPort::isOpen() const {
    return is_open_ && serial_ && serial_->is_open();
}

void SerialPort::setReceiveCallback(BytesCallback cb) {
    receive_callback_ = std::move(cb);
}

void SerialPort::startRead() {
    if (!is_open_ || !serial_) return;
    serial_->async_read_some(asio::buffer(read_buffer_), [this](const std::error_code& ec, size_t n) {
        if (ec) {
            // closed on purpose (close() cancels the pending read): leave quietly
            if (ec == asio::error::operation_aborted || ec == asio::error::bad_descriptor || !is_open_) return;
            std::cerr << "Serial read error: " << ec.message() << '\n';
            is_open_ = false;
            write_cv_.notify_all();
            return;
        }
        if (n > 0 && receive_callback_) receive_callback_(read_buffer_.data(), n);
        startRead();
    });
}

// An asynchronous read, so close() can cancel it: a blocking read_some() is not woken by closing the port from another
// thread, which made close() (and so Disconnect) hang until the module happened to send a byte.
void SerialPort::readThreadLoop() {
    startRead();
    io_context_->run();
}

void SerialPort::writeRC(const std::vector<uint8_t>& data) {
    if (!is_open_) return;
    
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        pending_rc_frame_ = data;
        rc_pending_ = true;
    }
    write_cv_.notify_one();
}

void SerialPort::writeCommand(const std::vector<uint8_t>& data) {
    if (!is_open_) return;
    
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        command_queue_.push_back(data);
    }
    write_cv_.notify_one();
}

void SerialPort::writeThreadLoop() {
    while (is_open_) {
        std::vector<uint8_t> buffer_to_write;
        
        {
            std::unique_lock<std::mutex> lock(write_mutex_);
            write_cv_.wait(lock, [this]() {
                return !is_open_ || rc_pending_ || !command_queue_.empty();
            });
            
            if (!is_open_) break;
            
            if (rc_pending_) {
                buffer_to_write = std::move(pending_rc_frame_);
                rc_pending_ = false;
            } else if (!command_queue_.empty()) {
                buffer_to_write = std::move(command_queue_.front());
                command_queue_.pop_front();
            }
        }
        
        if (!buffer_to_write.empty() && serial_ && serial_->is_open()) {
            std::error_code ec;
            asio::write(*serial_, asio::buffer(buffer_to_write), ec);
            if (ec && ec != asio::error::operation_aborted) {
                std::cerr << "Serial write error: " << ec.message() << "\n";
                is_open_ = false;
            }
        }
    }
}

void SerialPort::doNextWrite() {
    // This function is no longer used but kept empty for safety
}

} // namespace crsf

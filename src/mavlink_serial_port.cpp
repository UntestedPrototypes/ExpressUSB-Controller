#include "mavlink_controller/serial_port.h"
#include <iostream>

namespace mavlink_ctrl {

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
    is_open_ = false;

    // Wake up the write thread so it can exit
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_cv_.notify_all();
    }

    if (io_context_) {
        if (serial_ && serial_->is_open()) {
            std::error_code ec;
            serial_->cancel(ec);
            serial_->close(ec);
        }
        io_context_->stop();
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
    command_queue_.clear();
}

bool SerialPort::isOpen() const {
    return is_open_ && serial_ && serial_->is_open();
}

void SerialPort::setReceiveCallback(BytesCallback cb) {
    receive_callback_ = std::move(cb);
}

void SerialPort::readThreadLoop() {
    while (is_open_) {
        if (!serial_ || !serial_->is_open()) {
            break;
        }

        std::error_code ec;
        size_t bytes_transferred = serial_->read_some(asio::buffer(read_buffer_), ec);
        
        if (ec) {
            if (ec == asio::error::operation_aborted || ec == asio::error::bad_descriptor || !is_open_) {
                break;
            }
            std::cerr << "Serial read error: " << ec.message() << "\n";
            is_open_ = false;
            break;
        }

        if (bytes_transferred > 0 && receive_callback_) {
            receive_callback_(read_buffer_.data(), bytes_transferred);
        }
    }
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

} // namespace mavlink_ctrl

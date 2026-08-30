#include "crsf/parameters.h"
#include "crsf/crsf_protocol.h"
#include <iostream>
#include <cstring>
#include <sstream>

namespace crsf {

void ParameterStore::setReadRequestCallback(ReadRequestCallback cb) {
    std::unique_lock lock(mutex_);
    read_callback_ = cb;
}

void ParameterStore::beginReadAll(uint8_t total_count) {
    std::unique_lock lock(mutex_);
    total_count_ = total_count;
    current_request_index_ = 0;
    reading_all_ = true;
    read_complete_ = false;
    parameters_.clear();
    active_field_index_ = 0xFF;
    chunk_buffer_.clear();
    next_chunk_index_ = 0;
    
    if (total_count > 0 && read_callback_) {
        read_callback_(current_request_index_, 0);
    } else {
        read_complete_ = true;
        reading_all_ = false;
    }
}

bool ParameterStore::handleParameterEntry(uint8_t field_index, uint8_t chunks_remaining,
                                           const uint8_t* data, size_t data_len) {
    std::unique_lock lock(mutex_);
    
    if (field_index != active_field_index_) {
        active_field_index_ = field_index;
        chunk_buffer_.clear();
        next_chunk_index_ = 0; // Or keep track based on requests, simplified here.
    }
    
    chunk_buffer_.insert(chunk_buffer_.end(), data, data + data_len);
    
    if (chunks_remaining > 0) {
        next_chunk_index_++;
        if (read_callback_) {
            read_callback_(field_index, next_chunk_index_);
        }
        return true;
    } else {
        parseParameterPayload(field_index, chunk_buffer_.data(), chunk_buffer_.size());
        
        if (reading_all_) {
            current_request_index_++;
            if (current_request_index_ < total_count_) {
                if (read_callback_) {
                    read_callback_(current_request_index_, 0);
                }
                return true;
            } else {
                read_complete_ = true;
                reading_all_ = false;
                return false;
            }
        }
        return false;
    }
}

std::vector<Parameter> ParameterStore::getAll() const {
    std::shared_lock lock(mutex_);
    std::vector<Parameter> res;
    res.reserve(parameters_.size());
    for (const auto& kv : parameters_) {
        res.push_back(kv.second);
    }
    return res;
}

std::optional<Parameter> ParameterStore::get(uint8_t index) const {
    std::shared_lock lock(mutex_);
    auto it = parameters_.find(index);
    if (it != parameters_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool ParameterStore::isReadComplete() const {
    std::shared_lock lock(mutex_);
    return read_complete_;
}

void ParameterStore::reset() {
    std::unique_lock lock(mutex_);
    parameters_.clear();
    total_count_ = 0;
    current_request_index_ = 0;
    active_field_index_ = 0xFF;
    next_chunk_index_ = 0;
    chunk_buffer_.clear();
    reading_all_ = false;
    read_complete_ = false;
}

void ParameterStore::requestNextParameter() {
    if (reading_all_ && current_request_index_ < total_count_) {
        if (read_callback_) {
            read_callback_(current_request_index_, 0);
        }
    }
}

void ParameterStore::parseParameterPayload(uint8_t field_index, const uint8_t* data, size_t len) {
    if (len < 3) {
        std::cerr << "[CRSF Parser] Parameter payload too short for field " << (int)field_index << "\n";
        return;
    }
    
    Parameter param;
    param.index = field_index;
    param.parent = data[0];
    param.hidden = (data[1] & 0x80) != 0;
    param.type = data[1] & 0x7F;
    param.raw_data.assign(data, data + len);
    
    size_t offset = 2;
    // Extract label
    while (offset < len && data[offset] != '\0') {
        param.label.push_back(static_cast<char>(data[offset]));
        offset++;
    }
    if (offset < len) offset++; // skip null terminator
    
    auto read_string = [&](std::string& str) {
        while (offset < len && data[offset] != '\0') {
            str.push_back(static_cast<char>(data[offset]));
            offset++;
        }
        if (offset < len) offset++; // skip null
    };

    auto read_u8 = [&]() -> uint8_t {
        if (offset < len) return data[offset++];
        return 0;
    };
    
    auto read_u16_be = [&]() -> uint16_t {
        if (offset + 1 < len) {
            uint16_t val = (static_cast<uint16_t>(data[offset]) << 8) | data[offset+1];
            offset += 2;
            return val;
        }
        return 0;
    };
    
    auto read_u32_be = [&]() -> uint32_t {
        if (offset + 3 < len) {
            uint32_t val = (static_cast<uint32_t>(data[offset]) << 24) |
                           (static_cast<uint32_t>(data[offset+1]) << 16) |
                           (static_cast<uint32_t>(data[offset+2]) << 8) |
                           data[offset+3];
            offset += 4;
            return val;
        }
        return 0;
    };

    if (param.type == 0 || param.type == 1) { // UINT8 / INT8
        NumericParam num;
        num.value = (param.type == 1) ? static_cast<int8_t>(read_u8()) : read_u8();
        num.min_val = (param.type == 1) ? static_cast<int8_t>(read_u8()) : read_u8();
        num.max_val = (param.type == 1) ? static_cast<int8_t>(read_u8()) : read_u8();
        num.default_val = (param.type == 1) ? static_cast<int8_t>(read_u8()) : read_u8();
        read_string(num.units);
        param.numeric = num;
    } else if (param.type == 2 || param.type == 3) { // UINT16 / INT16
        NumericParam num;
        num.value = (param.type == 3) ? static_cast<int16_t>(read_u16_be()) : read_u16_be();
        num.min_val = (param.type == 3) ? static_cast<int16_t>(read_u16_be()) : read_u16_be();
        num.max_val = (param.type == 3) ? static_cast<int16_t>(read_u16_be()) : read_u16_be();
        num.default_val = (param.type == 3) ? static_cast<int16_t>(read_u16_be()) : read_u16_be();
        read_string(num.units);
        param.numeric = num;
    } else if (param.type == 8) { // FLOAT
        NumericParam num;
        num.value = static_cast<int32_t>(read_u32_be());
        num.min_val = static_cast<int32_t>(read_u32_be());
        num.max_val = static_cast<int32_t>(read_u32_be());
        num.default_val = static_cast<int32_t>(read_u32_be());
        num.decimal_places = read_u8();
        num.step = read_u32_be();
        read_string(num.units);
        param.numeric = num;
    } else if (param.type == 9) { // TEXT_SELECTION
        SelectParam sel;
        std::string options_str;
        read_string(options_str);
        
        std::stringstream ss(options_str);
        std::string token;
        while (std::getline(ss, token, ';')) {
            sel.options.push_back(token);
        }
        
        sel.current_index = read_u8();
        sel.min_val = read_u8();
        sel.max_val = read_u8();
        sel.default_val = read_u8();
        read_string(sel.units);
        param.select = sel;
    } else if (param.type == 10) { // STRING
        StringParam str_val;
        read_string(str_val.value);
        str_val.max_length = read_u8();
        param.string_val = str_val;
    } else if (param.type == 11) { // FOLDER
        // nothing extra
    } else if (param.type == 12) { // INFO
        StringParam str_val;
        read_string(str_val.value);
        param.string_val = str_val;
    } else if (param.type == 13) { // COMMAND
        // status(1), timeout(1), dialog-string
        NumericParam num;
        num.value = read_u8();
        num.max_val = read_u8(); // timeout
        read_string(num.units); // dialog string
        param.numeric = num;
    }
    
    parameters_[field_index] = param;
}

} // namespace crsf

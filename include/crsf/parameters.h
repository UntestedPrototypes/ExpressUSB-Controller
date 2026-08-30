#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <map>
#include <shared_mutex>
#include <functional>

namespace crsf {

struct NumericParam {
    int64_t value = 0;
    int64_t min_val = 0;
    int64_t max_val = 0;
    int64_t default_val = 0;
    std::string units;
    // For FLOAT type:
    uint8_t decimal_places = 0;
    uint32_t step = 0;
};

struct SelectParam {
    uint8_t current_index = 0;
    uint8_t min_val = 0;
    uint8_t max_val = 0;
    uint8_t default_val = 0;
    std::vector<std::string> options;
    std::string units;
};

struct StringParam {
    std::string value;
    uint8_t max_length = 0;
};

struct Parameter {
    uint8_t index = 0;
    uint8_t parent = 0;
    uint8_t type = 0;           // Low 7 bits of type_and_hidden
    bool hidden = false;        // MSB of type_and_hidden
    std::string label;
    std::vector<uint8_t> raw_data;  // Full raw payload for debugging

    std::optional<NumericParam> numeric;
    std::optional<SelectParam> select;
    std::optional<StringParam> string_val;
};

class ParameterStore {
public:
    using WriteRequestCallback = std::function<void(const std::vector<uint8_t>&)>;
    using ReadRequestCallback = std::function<void(uint8_t param_index, uint8_t chunk_index)>;

    // Set callback for when a parameter read request needs to be sent
    void setReadRequestCallback(ReadRequestCallback cb);
    
    // Start reading all parameters (called after DeviceInfo provides the count)
    void beginReadAll(uint8_t total_count);
    
    // Handle an incoming parameter settings entry frame (0x2B)
    // payload points to the data AFTER dest/orig addresses
    // Returns true if more chunks/parameters need to be requested
    bool handleParameterEntry(uint8_t field_index, uint8_t chunks_remaining,
                               const uint8_t* data, size_t data_len);
    
    // Get all parameters (thread-safe copy)
    std::vector<Parameter> getAll() const;
    
    // Get a specific parameter by index
    std::optional<Parameter> get(uint8_t index) const;
    
    // Check if all parameters have been read
    bool isReadComplete() const;
    
    // Reset all state
    void reset();

private:
    mutable std::shared_mutex mutex_;
    std::map<uint8_t, Parameter> parameters_;
    ReadRequestCallback read_callback_;
    
    // Chunk assembly state
    uint8_t total_count_ = 0;
    uint8_t current_request_index_ = 0;
    uint8_t active_field_index_ = 0xFF;
    uint8_t next_chunk_index_ = 0;
    std::vector<uint8_t> chunk_buffer_;
    bool reading_all_ = false;
    bool read_complete_ = false;
    
    // Parse assembled parameter payload
    void parseParameterPayload(uint8_t field_index, const uint8_t* data, size_t len);
    
    // Request next parameter
    void requestNextParameter();
};

} // namespace crsf

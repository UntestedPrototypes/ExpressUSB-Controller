#pragma once
#include <cstdint>
#include <vector>
#include <functional>
#include <common/mavlink.h>
#include "mavlink_controller/telemetry.h"

namespace mavlink_ctrl {

class MavlinkHandler {
public:
    using MessageCallback = std::function<void(const mavlink_message_t&)>;

    MavlinkHandler(Telemetry& telemetry);

    // Configuration
    void setGcsIds(uint8_t system_id, uint8_t component_id);
    void setTargetIds(uint8_t system_id, uint8_t component_id);

    // Set callback for every parsed message (for user's onMessage hook)
    void setMessageCallback(MessageCallback cb);

    // Feed incoming bytes from serial port
    void processByte(uint8_t byte);
    void processBytes(const uint8_t* data, size_t len);

    // Build outgoing messages
    std::vector<uint8_t> buildHeartbeat();
    std::vector<uint8_t> buildRcOverride(const uint16_t channels[18]);
    std::vector<uint8_t> buildCommandLong(uint16_t command, float p1=0, float p2=0, float p3=0, float p4=0, float p5=0, float p6=0, float p7=0);

    // Check if we've received a heartbeat from the target
    bool hasRemoteHeartbeat() const;

private:
    void handleMessage(const mavlink_message_t& msg);

    Telemetry& telemetry_;
    MessageCallback message_cb_;

    uint8_t gcs_system_id_ = 255;
    uint8_t gcs_component_id_ = 190;  // MAV_COMP_ID_MISSIONPLANNER
    uint8_t target_system_id_ = 1;
    uint8_t target_component_id_ = 1; // MAV_COMP_ID_AUTOPILOT1

    bool has_remote_heartbeat_ = false;
};

} // namespace mavlink_ctrl

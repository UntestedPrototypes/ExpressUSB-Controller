#include "mavlink_controller/mavlink_handler.h"
#include <chrono>
#include <cstring>

namespace mavlink_ctrl {

MavlinkHandler::MavlinkHandler(Telemetry& telemetry)
    : telemetry_(telemetry) {}

void MavlinkHandler::setGcsIds(uint8_t system_id, uint8_t component_id) {
    gcs_system_id_ = system_id;
    gcs_component_id_ = component_id;
}

void MavlinkHandler::setTargetIds(uint8_t system_id, uint8_t component_id) {
    target_system_id_ = system_id;
    target_component_id_ = component_id;
}

void MavlinkHandler::setMessageCallback(MessageCallback cb) {
    message_cb_ = cb;
}

void MavlinkHandler::processByte(uint8_t byte) {
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status) == MAVLINK_FRAMING_OK) {
        handleMessage(msg);
        if (message_cb_) {
            message_cb_(msg);
        }
    }
}

void MavlinkHandler::processBytes(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        processByte(data[i]);
    }
}

static uint32_t get_time_ms() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void MavlinkHandler::handleMessage(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (!has_remote_heartbeat_ || (msg.sysid == target_system_id_ && msg.compid == target_component_id_)) {
                has_remote_heartbeat_ = true;
                target_system_id_ = msg.sysid;
                target_component_id_ = msg.compid;
            }
            telemetry_.updateHeartbeat(msg.sysid, msg.compid, hb.type, hb.autopilot, hb.base_mode, hb.custom_mode, hb.system_status, get_time_ms());
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&msg, &sys);
            telemetry_.updateSysStatus(sys.voltage_battery, sys.current_battery, sys.battery_remaining, sys.load, sys.drop_rate_comm, sys.errors_comm);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);
            telemetry_.updateGpsRaw(gps.fix_type, gps.lat, gps.lon, gps.alt, gps.eph, gps.epv, gps.vel, gps.satellites_visible);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);
            telemetry_.updateAttitude(att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed);
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            telemetry_.updateGlobalPosition(pos.lat, pos.lon, pos.alt, pos.relative_alt, pos.vx, pos.vy, pos.vz, pos.hdg);
            break;
        }
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
            mavlink_servo_output_raw_t servo;
            mavlink_msg_servo_output_raw_decode(&msg, &servo);
            uint16_t servos[16] = { servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw,
                                    servo.servo5_raw, servo.servo6_raw, servo.servo7_raw, servo.servo8_raw,
                                    servo.servo9_raw, servo.servo10_raw, servo.servo11_raw, servo.servo12_raw,
                                    servo.servo13_raw, servo.servo14_raw, servo.servo15_raw, servo.servo16_raw };
            telemetry_.updateServoOutputRaw(servos);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t rc;
            mavlink_msg_rc_channels_decode(&msg, &rc);
            uint16_t channels[18] = { rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw, rc.chan5_raw, rc.chan6_raw,
                                      rc.chan7_raw, rc.chan8_raw, rc.chan9_raw, rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
                                      rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw, rc.chan17_raw, rc.chan18_raw };
            telemetry_.updateRcChannels(channels, rc.chancount, rc.rssi);
            break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t vfr;
            mavlink_msg_vfr_hud_decode(&msg, &vfr);
            telemetry_.updateVfrHud(vfr.airspeed, vfr.groundspeed, vfr.heading, vfr.throttle, vfr.alt, vfr.climb);
            break;
        }
        case MAVLINK_MSG_ID_RADIO_STATUS: {
            mavlink_radio_status_t radio;
            mavlink_msg_radio_status_decode(&msg, &radio);
            telemetry_.updateRadioStatus(radio.rssi, radio.remrssi, radio.txbuf, radio.noise, radio.remnoise, radio.rxerrors, radio.fixed);
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
            mavlink_named_value_float_t nvf;
            mavlink_msg_named_value_float_decode(&msg, &nvf);
            char name[11] = {0};
            std::memcpy(name, nvf.name, 10);
            telemetry_.updateNamedValueFloat(name, nvf.value);
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
            mavlink_named_value_int_t nvi;
            mavlink_msg_named_value_int_decode(&msg, &nvi);
            char name[11] = {0};
            std::memcpy(name, nvi.name, 10);
            telemetry_.updateNamedValueInt(name, nvi.value);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t st;
            mavlink_msg_statustext_decode(&msg, &st);
            char text[51] = {0};
            std::memcpy(text, st.text, 50);
            telemetry_.updateStatustext(text, st.severity);
            break;
        }
        default:
            break;
    }
}

std::vector<uint8_t> MavlinkHandler::buildHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(gcs_system_id_, gcs_component_id_, &msg, 
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &msg);
    buf.resize(len);
    return buf;
}

std::vector<uint8_t> MavlinkHandler::buildRcOverride(const uint16_t channels[18]) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_override_pack(gcs_system_id_, gcs_component_id_, &msg,
        target_system_id_, target_component_id_,
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7],
        channels[8], channels[9], channels[10], channels[11],
        channels[12], channels[13], channels[14], channels[15],
        channels[16], channels[17]);

    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &msg);
    buf.resize(len);
    return buf;
}

std::vector<uint8_t> MavlinkHandler::buildCommandLong(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(gcs_system_id_, gcs_component_id_, &msg,
        target_system_id_, target_component_id_, command, 0,
        p1, p2, p3, p4, p5, p6, p7);

    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(buf.data(), &msg);
    buf.resize(len);
    return buf;
}

bool MavlinkHandler::hasRemoteHeartbeat() const {
    return has_remote_heartbeat_;
}

} // namespace mavlink_ctrl

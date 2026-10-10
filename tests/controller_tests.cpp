// Self-tests for the link modes. No hardware: the controller talks to a loopback transport.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include "mavlink_controller/crsf_handler.h"
#include "mavlink_controller/crsf_protocol.h"
#include "mavlink_controller/mavlink_controller.h"
#include "mavlink_controller/transport.h"

#ifdef EXPRESSUSB_TEST_PENTAPOD_INTEROP
#include "drivers/crsf_parser.h"   // the robot's own CRSF parser (PentaPod), read only
#endif

using namespace mavlink_ctrl;
using Clock = std::chrono::steady_clock;

static int g_failures = 0;
#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("  FAIL line %d: %s\n", __LINE__, #cond);          \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)
#define CHECK_EQ(a, b)                                                                                  \
    do {                                                                                                \
        const long long va = (long long)(a), vb = (long long)(b);                                       \
        if (va != vb) {                                                                                 \
            std::printf("  FAIL line %d: %s == %s  (%lld vs %lld)\n", __LINE__, #a, #b, va, vb);        \
            ++g_failures;                                                                               \
        }                                                                                               \
    } while (0)

static bool waitFor(const std::function<bool()>& pred, int timeout_ms = 2000) {
    const auto end = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (Clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

// ------------------------------------------------------------------ loopback transport

class LoopbackTransport : public ITransport {
public:
    bool open(const std::string& port, unsigned int baud) override {
        std::lock_guard<std::mutex> l(m);
        opened_port = port;
        opened_baud = baud;
        ++open_count;
        is_open = true;
        return true;
    }
    void close() override {
        std::lock_guard<std::mutex> l(m);
        is_open = false;
    }
    bool isOpen() const override {
        std::lock_guard<std::mutex> l(m);
        return is_open;
    }
    void setReceiveCallback(BytesCallback cb) override {
        std::lock_guard<std::mutex> l(m);
        rx = std::move(cb);
    }
    void writeRC(const std::vector<uint8_t>& d) override {
        std::lock_guard<std::mutex> l(m);
        rc_frames.push_back(d);
    }
    void writeCommand(const std::vector<uint8_t>& d) override {
        std::lock_guard<std::mutex> l(m);
        command_frames.push_back(d);
    }

    // As if the serial read thread had received these bytes
    void inject(const std::vector<uint8_t>& bytes) {
        BytesCallback cb;
        {
            std::lock_guard<std::mutex> l(m);
            cb = rx;
        }
        if (cb) cb(bytes.data(), bytes.size());
    }
    size_t rcCount() const { std::lock_guard<std::mutex> l(m); return rc_frames.size(); }
    size_t cmdCount() const { std::lock_guard<std::mutex> l(m); return command_frames.size(); }
    std::vector<uint8_t> lastRc() const { std::lock_guard<std::mutex> l(m); return rc_frames.empty() ? std::vector<uint8_t>() : rc_frames.back(); }
    std::vector<uint8_t> firstCmd() const { std::lock_guard<std::mutex> l(m); return command_frames.empty() ? std::vector<uint8_t>() : command_frames.front(); }

    mutable std::mutex m;
    std::string opened_port;
    unsigned opened_baud = 0;
    int open_count = 0;
    bool is_open = false;
    BytesCallback rx;
    std::vector<std::vector<uint8_t>> rc_frames, command_frames;
};

// A telemetry frame the way the TX module sends it to the handset
static std::vector<uint8_t> telemetryFrame(uint8_t type, const std::vector<uint8_t>& payload) {
    return crsf::buildFrame(crsf::ADDR_RADIO_TRANSMITTER, type, payload.data(), payload.size());
}
static std::vector<uint8_t> linkStatsFrame(uint8_t lq, int8_t snr = 10) {
    return telemetryFrame(crsf::TYPE_LINK_STATISTICS, {40, 45, lq, uint8_t(snr), 1, 3, 4, 50, 97, uint8_t(int8_t(8))});
}

// ------------------------------------------------------------------ tests

static void test_crsf_protocol_basics() {
    std::printf("crsf protocol helpers\n");
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_EQ(crsf::crc8(check, sizeof(check)), 0xBC);   // CRC-8/DVB-S2 check value

    uint16_t mid[16];
    for (auto& v : mid) v = 992;
    uint8_t packed[22];
    crsf::packChannels(mid, packed);
    const uint8_t expect[11] = {0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C};   // the familiar all-centre frame
    CHECK(std::memcmp(packed, expect, 11) == 0);
    CHECK(std::memcmp(packed + 11, expect, 11) == 0);

    std::mt19937 rng(5);
    for (int n = 0; n < 200; ++n) {
        uint16_t in[16], out[16];
        for (auto& v : in) v = uint16_t(rng() & 0x7FF);
        crsf::packChannels(in, packed);
        crsf::unpackChannels(packed, out);
        CHECK(std::memcmp(in, out, sizeof(in)) == 0);
    }

    CHECK_EQ(crsf::usToValue(1500), 992);
    CHECK_EQ(crsf::usToValue(1000), 192);
    CHECK_EQ(crsf::usToValue(2000), 1792);
    CHECK_EQ(crsf::usToValue(500), crsf::VALUE_MIN);    // clamped to the legal range
    CHECK_EQ(crsf::usToValue(3000), crsf::VALUE_MAX);
    for (uint16_t us = 990; us <= 2010; us += 7) {
        const int diff = int(crsf::valueToUs(crsf::usToValue(us))) - int(us);
        CHECK(diff >= -1 && diff <= 1);
    }

    const auto f = crsf::buildRcChannelsFrame(mid);
    CHECK_EQ(f.size(), 26);
    CHECK_EQ(f[0], crsf::ADDR_CRSF_TRANSMITTER);
    CHECK_EQ(f[1], 24);
    CHECK_EQ(f[2], crsf::TYPE_RC_CHANNELS);
    CHECK_EQ(f.back(), crsf::crc8(f.data() + 2, 23));
}

#ifdef EXPRESSUSB_TEST_PENTAPOD_INTEROP
static void test_interop_with_the_robots_crsf_parser() {
    std::printf("RC frames -> PentaPod's robot-side CRSF parser\n");
    uint16_t us[18];
    for (auto& v : us) v = UINT16_MAX;
    us[0] = 1000; us[1] = 1250; us[2] = 1500; us[3] = 1750; us[4] = 2000; us[5] = 1100; us[8] = 1900;
    Telemetry tel;
    CrsfHandler handler(tel);
    const auto frame = handler.buildRcFrame(us);

    pentapod::CrsfParser parser;
    pentapod::CrsfParser::Event e = pentapod::CrsfParser::None;
    for (uint8_t b : frame) e = parser.feed(b);
    CHECK_EQ(e, pentapod::CrsfParser::Channels);
    CHECK_EQ(parser.channels()[0], crsf::usToValue(1000));
    CHECK_EQ(parser.channels()[2], 992);
    CHECK_EQ(parser.channels()[4], crsf::usToValue(2000));
    CHECK_EQ(parser.channels()[8], crsf::usToValue(1900));
    CHECK_EQ(parser.channels()[7], 992);           // never set: centred
    CHECK_EQ(parser.crcErrors(), 0);
    // the robot's stick scale: 2000 us is about +100 % deflection, 1500 us zero
    CHECK(pentapod::CrsfParser::normalized(parser.channels()[4]) > 0.97f);
    CHECK(pentapod::CrsfParser::normalized(parser.channels()[2]) == 0.0f);
}
#endif

static void test_crsf_telemetry_decoding() {
    std::printf("crsf telemetry decoding\n");
    Telemetry tel;
    CrsfHandler h(tel);
    std::vector<uint8_t> seenTypes;
    h.setFrameCallback([&](uint8_t type, const uint8_t*, size_t) { seenTypes.push_back(type); });

    std::vector<uint8_t> stream;
    auto append = [&](const std::vector<uint8_t>& f) { stream.insert(stream.end(), f.begin(), f.end()); };

    stream.push_back(0x55); stream.push_back(0x01);   // line noise before the first frame
    append(linkStatsFrame(98));
    append(telemetryFrame(crsf::TYPE_BATTERY, {0x00, 0x4A, 0x00, 0x15, 0x00, 0x03, 0xE8, 85}));       // 7.4 V, 2.1 A, 1000 mAh, 85 %
    append(telemetryFrame(crsf::TYPE_ATTITUDE, {0x00, 0x64, 0xFF, 0x9C, 0x1B, 0x58}));                // pitch 0.01, roll -0.01, yaw 0.7
    // GPS: lat 52.0, lon 4.0, 36 km/h, heading 90.00 deg, altitude 1012 m (raw 2012), 9 satellites
    append(telemetryFrame(crsf::TYPE_GPS, {0x1E, 0xFE, 0x92, 0x00, 0x02, 0x62, 0x5A, 0x00, 0x01, 0x68, 0x23, 0x28, 0x07, 0xDC, 9}));
    std::vector<uint8_t> mode;
    for (char c : std::string("LIMIT S08 L3")) mode.push_back(uint8_t(c));
    mode.push_back(0);
    append(telemetryFrame(crsf::TYPE_FLIGHT_MODE, mode));
    append(telemetryFrame(0x7F, {1, 2, 3}));           // a type nobody decodes: still reaches the tap

    // a frame with a damaged checksum, then a good one: the stream recovers
    auto bad = telemetryFrame(crsf::TYPE_BATTERY, {0x00, 0x63, 0, 0, 0, 0, 0, 10});
    bad.back() ^= 0x5A;
    append(bad);
    append(telemetryFrame(crsf::TYPE_BATTERY, {0x00, 0x4B, 0x00, 0x15, 0x00, 0x03, 0xE8, 84}));      // 7.5 V

    // fed one byte at a time, the hardest case for the parser
    for (uint8_t b : stream) h.processBytes(&b, 1);

    auto d = tel.get();
    CHECK(d.crsf_link_valid);
    CHECK_EQ(d.crsf_uplink_lq, 98);
    CHECK_EQ(d.crsf_uplink_rssi_1, 40);
    CHECK_EQ(d.crsf_uplink_snr, 10);
    CHECK_EQ(d.crsf_downlink_lq, 97);
    CHECK_EQ(d.crsf_downlink_snr, 8);
    CHECK_EQ(d.radio_rssi, 250);                      // 98 % of 255, like the ELRS receiver reports it
    CHECK(d.last_heartbeat_ms > 0);
    CHECK_EQ(d.battery_voltage_mv, 7500);             // the second (good) battery frame won
    CHECK_EQ(d.battery_current_ca, 210);
    CHECK_EQ(d.battery_remaining, 84);
    CHECK(std::abs(d.attitude_pitch - 0.01f) < 1e-4f);
    CHECK(std::abs(d.attitude_roll + 0.01f) < 1e-4f);
    CHECK(std::abs(d.attitude_yaw - 0.7f) < 1e-3f);
    CHECK_EQ(d.gps_lat, 520000000);
    CHECK_EQ(d.gps_lon, 40000000);
    CHECK_EQ(d.gps_satellites, 9);
    CHECK_EQ(d.gps_fix_type, 3);
    CHECK_EQ(d.gps_vel, 1000);                        // 36 km/h = 1000 cm/s
    CHECK_EQ(d.gps_alt_mm, 1012000);
    CHECK(d.flight_mode == "LIMIT S08 L3");
    CHECK_EQ(h.crcErrors(), 1);
    CHECK_EQ(d.crsf_crc_errors, 1);
    CHECK(std::find(seenTypes.begin(), seenTypes.end(), 0x7F) != seenTypes.end());
    CHECK(h.linkUp());

    // a link quality of 0 means no link, even though frames keep arriving
    h.processBytes(linkStatsFrame(0).data(), linkStatsFrame(0).size());
    CHECK(!h.linkUp());

    // junk that looks like the start of a frame (an address byte followed by an impossible length)
    h.reset();
    auto f = linkStatsFrame(90);
    const std::vector<uint8_t> junk = {crsf::ADDR_RADIO_TRANSMITTER, 0x90, 0x14};
    h.processBytes(junk.data(), junk.size());
    h.processBytes(f.data(), f.size());
    CHECK(h.linkUp());

    // junk that looks like a frame start with a plausible length swallows the next frame while it waits
    // for its checksum, then the stream recovers: one lost frame, not a lost connection
    h.reset();
    const std::vector<uint8_t> plausible = {crsf::ADDR_RADIO_TRANSMITTER, 0x0C, 0x14, 1, 2};
    h.processBytes(plausible.data(), plausible.size());
    for (int i = 0; i < 4; ++i) h.processBytes(f.data(), f.size());
    CHECK(h.linkUp());
    CHECK(h.crcErrors() >= 1);
}

static void test_crsf_channel_values() {
    std::printf("crsf channel handling\n");
    Telemetry tel;
    CrsfHandler h(tel);
    uint16_t us[18];
    for (auto& v : us) v = UINT16_MAX;

    auto values = [&](const std::vector<uint8_t>& f) {
        std::vector<uint16_t> v(16);
        crsf::unpackChannels(f.data() + 3, v.data());
        return v;
    };
    auto v = values(h.buildRcFrame(us));
    for (int i = 0; i < 16; ++i) CHECK_EQ(v[i], 992);   // nothing set: centred, so a robot sees "disarmed", sticks neutral

    us[0] = 2000; us[4] = 1000; us[16] = 1234; us[17] = 1234;   // channels 17/18 do not exist in CRSF
    v = values(h.buildRcFrame(us));
    CHECK_EQ(v[0], 1792);
    CHECK_EQ(v[4], 192);

    us[0] = UINT16_MAX; us[4] = 0;                      // "no value": the last values stay
    v = values(h.buildRcFrame(us));
    CHECK_EQ(v[0], 1792);
    CHECK_EQ(v[4], 192);

    h.reset();                                          // a new connection starts centred again
    v = values(h.buildRcFrame(us));
    CHECK_EQ(v[0], 992);
}

static void test_link_mode_names() {
    std::printf("link mode names\n");
    LinkMode m = LinkMode::Mavlink;
    CHECK(MavlinkController::parseLinkMode("CRSF", m) && m == LinkMode::Crsf);
    CHECK(MavlinkController::parseLinkMode("mavlink", m) && m == LinkMode::Mavlink);
    CHECK(MavlinkController::parseLinkMode("Mav", m) && m == LinkMode::Mavlink);
    CHECK(!MavlinkController::parseLinkMode("sbus", m));
    CHECK_EQ(MavlinkController::defaultBaud(LinkMode::Mavlink), 460800);
    CHECK_EQ(MavlinkController::defaultBaud(LinkMode::Crsf), 400000);
    CHECK(std::string(MavlinkController::linkModeName(LinkMode::Crsf)) == "crsf");
}

static void test_controller_crsf_mode() {
    std::printf("controller in CRSF mode\n");
    auto lb = std::make_unique<LoopbackTransport>();
    LoopbackTransport* t = lb.get();
    MavlinkController c;
    c.setTransport(std::move(lb));
    CHECK(c.getLinkMode() == LinkMode::Mavlink);   // the default is unchanged
    c.setLinkMode(LinkMode::Crsf);

    std::vector<ConnectionState> states;
    std::mutex sm;
    c.onStateChange([&](ConnectionState s) { std::lock_guard<std::mutex> l(sm); states.push_back(s); });
    std::vector<uint8_t> tappedTypes;
    c.onCrsfFrame([&](uint8_t type, const uint8_t*, size_t) { std::lock_guard<std::mutex> l(sm); tappedTypes.push_back(type); });

    CHECK(c.connect("TESTPORT"));
    CHECK_EQ(t->opened_baud, 400000);                // default baud of the mode
    CHECK(t->opened_port == "TESTPORT");

    c.setChannel(0, 2000);
    c.setChannel(2, 1000);
    c.setChannel(4, 2000);
    CHECK(waitFor([&] { return t->rcCount() > 20; }));

    // frames are RC channel frames with the values that were set
    const auto last = t->lastRc();
    CHECK_EQ(last.size(), 26);
    CHECK_EQ(last[0], crsf::ADDR_CRSF_TRANSMITTER);
    CHECK_EQ(last[2], crsf::TYPE_RC_CHANNELS);
    CHECK_EQ(last.back(), crsf::crc8(last.data() + 2, 23));
    uint16_t v[16];
    crsf::unpackChannels(last.data() + 3, v);
    CHECK_EQ(v[0], 1792);
    CHECK_EQ(v[1], 992);                             // never set
    CHECK_EQ(v[2], 192);
    CHECK_EQ(v[4], 1792);

    CHECK_EQ(t->cmdCount(), 0);                      // CRSF needs no heartbeat

    // the RC rate of the mode: 250 Hz by default
    const size_t before = t->rcCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const size_t rate = (t->rcCount() - before) * 1000 / 400;
    std::printf("  RC frame rate: about %zu Hz\n", rate);
    CHECK(rate > 80 && rate < 400);

    // link state follows the link statistics
    CHECK(c.getState() == ConnectionState::Connected);
    t->inject(linkStatsFrame(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    CHECK(c.getState() == ConnectionState::Connected);   // frames arrive, but no link to the receiver
    t->inject(linkStatsFrame(99));
    CHECK(waitFor([&] { return c.getState() == ConnectionState::LinkUp; }, 1000));
    t->inject(telemetryFrame(crsf::TYPE_BATTERY, {0x00, 0x4A, 0x00, 0x00, 0, 0, 0, 90}));
    t->inject(telemetryFrame(crsf::TYPE_FLIGHT_MODE, {'O', 'K', 0}));
    CHECK(waitFor([&] { return c.getTelemetry().battery_voltage_mv == 7400; }, 500));
    CHECK(waitFor([&] { return c.getTelemetry().flight_mode == "OK"; }, 500));
    CHECK_EQ(c.getTelemetry().crsf_uplink_lq, 99);
    {
        std::lock_guard<std::mutex> l(sm);
        CHECK(std::find(tappedTypes.begin(), tappedTypes.end(), crsf::TYPE_BATTERY) != tappedTypes.end());
    }

    // an explicit baud rate and RC rate win over the mode defaults
    const auto t0 = Clock::now();
    c.disconnect();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
    std::printf("  disconnect took %lld ms\n", (long long)ms);
    CHECK(ms < 500);                                   // the worker threads wake up at once
    CHECK(c.getState() == ConnectionState::Disconnected);
    CHECK(!c.isConnected());
    {
        std::lock_guard<std::mutex> l(sm);
        CHECK(states.front() == ConnectionState::Connected);
        CHECK(states.back() == ConnectionState::Disconnected);
    }

    c.setRcRate(10000);
    CHECK(c.connect("TESTPORT", 921600));
    CHECK_EQ(t->opened_baud, 921600);
    const size_t b2 = t->rcCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const size_t rate2 = (t->rcCount() - b2) * 1000 / 400;
    std::printf("  RC frame rate with setRcRate(10000): about %zu Hz\n", rate2);
    CHECK(rate2 > 50 && rate2 < 150);
    c.disconnect();
}

static void test_controller_mavlink_mode_is_unchanged() {
    std::printf("controller in MAVLink mode (regression)\n");
    auto lb = std::make_unique<LoopbackTransport>();
    LoopbackTransport* t = lb.get();
    MavlinkController c;
    c.setTransport(std::move(lb));

    CHECK(c.connect("TESTPORT"));
    CHECK_EQ(t->opened_baud, 460800);

    c.setChannel(0, 1750);
    c.setChannel(4, 2000);
    CHECK(waitFor([&] { return t->cmdCount() >= 1 && t->rcCount() >= 5; }));

    // the GCS heartbeat
    mavlink_message_t msg;
    mavlink_status_t status;
    bool got = false;
    for (uint8_t b : t->firstCmd())
        if (mavlink_parse_char(MAVLINK_COMM_2, b, &msg, &status) == MAVLINK_FRAMING_OK) got = true;
    CHECK(got && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    CHECK_EQ(hb.type, MAV_TYPE_GCS);

    // the RC override
    got = false;
    for (uint8_t b : t->lastRc())
        if (mavlink_parse_char(MAVLINK_COMM_2, b, &msg, &status) == MAVLINK_FRAMING_OK) got = true;
    CHECK(got && msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE);
    mavlink_rc_channels_override_t rc;
    mavlink_msg_rc_channels_override_decode(&msg, &rc);
    CHECK_EQ(rc.chan1_raw, 1750);
    CHECK_EQ(rc.chan5_raw, 2000);
    CHECK_EQ(rc.chan2_raw, UINT16_MAX);                 // never set: ignored

    // RC rate 50 Hz
    const size_t before = t->rcCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const size_t rate = (t->rcCount() - before) * 1000 / 500;
    std::printf("  RC frame rate: about %zu Hz\n", rate);
    CHECK(rate > 25 && rate < 80);

    // link state follows the remote heartbeat; telemetry is decoded
    CHECK(c.getState() == ConnectionState::Connected);
    mavlink_message_t rhb;
    mavlink_msg_heartbeat_pack(1, 1, &rhb, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC, 129, 0x01020304, MAV_STATE_ACTIVE);
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    buf.resize(mavlink_msg_to_send_buffer(buf.data(), &rhb));
    t->inject(buf);
    CHECK(waitFor([&] { return c.getState() == ConnectionState::LinkUp; }, 1000));
    mavlink_message_t nv;
    mavlink_msg_named_value_int_pack(1, 1, &nv, 100, "FAULT", 3);
    buf.resize(MAVLINK_MAX_PACKET_LEN);
    buf.resize(mavlink_msg_to_send_buffer(buf.data(), &nv));
    t->inject(buf);
    CHECK(waitFor([&] { return c.getTelemetry().named_values_int.count("FAULT") > 0; }, 500));
    CHECK_EQ(c.getTelemetry().named_values_int["FAULT"], 3);
    CHECK_EQ(c.getTelemetry().remote_custom_mode, 0x01020304);

    // the raw message tap
    std::atomic<int> tapped{0};
    c.disconnect();
    c.onMessage([&](const mavlink_message_t&) { ++tapped; });
    CHECK(c.connect("TESTPORT"));
    t->inject(buf);
    CHECK(waitFor([&] { return tapped.load() >= 1; }, 500));

    const auto t0 = Clock::now();
    c.disconnect();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
    std::printf("  disconnect took %lld ms\n", (long long)ms);
    CHECK(ms < 500);   // used to wait for the sleeping heartbeat thread, up to a second
}

static void test_switching_modes() {
    std::printf("switching modes on one controller\n");
    auto lb = std::make_unique<LoopbackTransport>();
    LoopbackTransport* t = lb.get();
    MavlinkController c;
    c.setTransport(std::move(lb));

    c.setLinkMode(LinkMode::Crsf);
    CHECK(c.connect("P"));
    t->inject(linkStatsFrame(95));
    CHECK(waitFor([&] { return c.getState() == ConnectionState::LinkUp; }, 1000));
    CHECK(waitFor([&] { return t->rcCount() > 3; }));
    CHECK_EQ(t->lastRc()[0], crsf::ADDR_CRSF_TRANSMITTER);
    c.disconnect();

    c.setLinkMode(LinkMode::Mavlink);
    t->rc_frames.clear();
    t->command_frames.clear();
    CHECK(c.connect("P"));
    CHECK_EQ(t->opened_baud, 460800);
    CHECK(c.getState() == ConnectionState::Connected);              // the CRSF link does not carry over
    CHECK(c.getTelemetry().crsf_uplink_lq == 0 && !c.getTelemetry().crsf_link_valid);   // and neither does its telemetry
    CHECK(waitFor([&] { return t->rcCount() > 3 && t->cmdCount() > 0; }));
    CHECK_EQ(t->lastRc()[0], 0xFD);                                  // MAVLink 2 frames now
    c.disconnect();

    c.setLinkMode(LinkMode::Crsf);
    CHECK(c.connect("P"));
    CHECK_EQ(t->opened_baud, 400000);
    c.disconnect();
}

int main() {
    test_crsf_protocol_basics();
#ifdef EXPRESSUSB_TEST_PENTAPOD_INTEROP
    test_interop_with_the_robots_crsf_parser();
#else
    std::printf("(PentaPod interoperability test skipped: PentaPod sources not found)\n");
#endif
    test_crsf_telemetry_decoding();
    test_crsf_channel_values();
    test_link_mode_names();
    test_controller_crsf_mode();
    test_controller_mavlink_mode_is_unchanged();
    test_switching_modes();
    std::printf(g_failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}

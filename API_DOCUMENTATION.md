# ExpressUSB MAVLink Controller - API Documentation

The `ExpressUSB` library provides a robust, cross-platform C++20 interface for controlling and receiving telemetry from an ExpressLRS TX module, in one of two **link modes** that you choose:

| Mode | The TX module runs | What the PC sends | What comes back | Default baud |
|:---|:---|:---|:---|:---|
| `LinkMode::Mavlink` (default) | Native MAVLink | `RC_CHANNELS_OVERRIDE` + GCS heartbeat | MAVLink messages | 460800 |
| `LinkMode::Crsf` | normal (CRSF) mode, the PC is its handset | CRSF RC channels frame | CRSF telemetry frames | 400000 |

It handles all background serial I/O, message framing/parsing, automatic reconnections, and multi-threaded synchronization. `setChannel()`, `getTelemetry()`, `onStateChange()` and the rest of the API work the same way in both modes.

## Choosing the link mode

```cpp
mavlink_ctrl::MavlinkController controller;
controller.setLinkMode(mavlink_ctrl::LinkMode::Crsf);   // before connect(); default is Mavlink
controller.connect("COM10");                            // baud 0 (default) = the default of the mode

// Switching later: disconnect, change the mode, connect again.
controller.disconnect();
controller.setLinkMode(mavlink_ctrl::LinkMode::Mavlink);
controller.connect("COM10");

// From text, e.g. a command line option:
mavlink_ctrl::LinkMode m;
if (mavlink_ctrl::MavlinkController::parseLinkMode("crsf", m)) controller.setLinkMode(m);
```

The TX module (and receiver) must be set to the same mode in ExpressLRS, see `SETUP_GUI.md`. Telemetry from the previous connection (or from the other mode) is cleared when you connect.

### What differs between the modes

| | MAVLink | CRSF |
|:---|:---|:---|
| Heartbeat | GCS heartbeat every `setHeartbeatRate()` ms | none needed |
| RC rate (`setRcRate()`) | 20 ms (50 Hz) | 4 ms (250 Hz) |
| Channels | 18 (`UINT16_MAX` = ignore, 0 = release) | 16; 17 and 18 are ignored. `UINT16_MAX` and 0 keep the last value; centred (1500) until a value is set |
| `ConnectionState::LinkUp` | a remote heartbeat arrived | link statistics with a link quality above 0 arrived (within 3 s) |
| `onMessage()` | every MAVLink message | not called; use `onCrsfFrame()` |
| Battery, attitude, GPS, `radio_rssi` | MAVLink messages | the matching CRSF frames fill the same `TelemetryData` fields |
| Named values, status text | yes | no. The remote device's flight-mode text is in `flight_mode` |

### CRSF telemetry

```cpp
auto t = controller.getTelemetry();
t.crsf_link_valid;                       // link statistics received
t.crsf_uplink_lq;                        // 0-100 %
t.crsf_uplink_rssi_1;                    // -dBm
t.crsf_uplink_snr;                       // dB
t.crsf_downlink_lq;
t.battery_voltage_mv;                    // from the battery frame, same field as in MAVLink mode
t.flight_mode;                           // text the remote device sends as "flight mode"
t.radio_rssi;                            // uplink link quality scaled to 0-255 (as ELRS reports it in MAVLink mode)

// every valid CRSF frame, including types the library does not decode
controller.onCrsfFrame([](uint8_t type, const uint8_t* payload, size_t length) { /* ... */ });
```

Decoded frame types: link statistics (0x14), battery (0x08), GPS (0x02), attitude (0x1E), flight mode (0x21). The RC frame is built by `crsf::buildRcChannelsFrame()` (`crsf_protocol.h`, header-only; also has the CRC and the microsecond conversion).

### Timing note (Windows)

Standard sleeps on Windows wake up in steps of about 15.6 ms. The RC thread therefore uses a high-resolution waitable timer, which keeps 250 Hz accurate. (Earlier versions of this library sent "50 Hz" MAVLink RC at about 32 Hz on Windows.)

### Self-tests

`ctest` (or `controller_tests`) runs without hardware: CRSF framing, telemetry decoding, both modes of the controller against a loopback transport, and switching between them. If the `PentaPod` project sits next to this one, the test also checks that its robot-side CRSF parser decodes the frames this library sends.

## Core Architecture

The architecture consists of three main components:
1. `MavlinkController`: The main public API for connecting, controlling, and accessing telemetry.
2. `TelemetryData`: A thread-safe data store containing the latest snapshot of all received MAVLink messages.
3. `MavlinkHandler` & `SerialPort`: Internal components managing byte processing, message routing, and OS-level COM ports.

---

## 1. `mavlink_ctrl::MavlinkController`

The primary class you will instantiate to interface with the radio.

### Initialization & Configuration

```cpp
mavlink_ctrl::MavlinkController controller;

// (Optional) Change the target System ID and Component ID we are talking to.
// Defaults: System 1, Component 1 (Standard Autopilot)
controller.setTargetSystem(1, 1);

// (Optional) Change our Ground Control Station IDs.
// Defaults: System 255, Component 190 (Mission Planner)
controller.setGcsIds(255, 190);

// (Optional) Configure background thread rates
controller.setHeartbeatRate(1000); // 1Hz GCS heartbeat (Default)
controller.setRcRate(20000);       // RC frame interval in microseconds. Default: 20000 (50 Hz) in MAVLink mode, 4000 (250 Hz) in CRSF mode

// (Optional) Enable auto-reconnection if USB is unplugged
controller.setAutoReconnect(true, 2000 /* ms */);
```

### Connection Management

```cpp
// List available serial ports (e.g. COM1, /dev/ttyUSB0)
std::vector<std::string> ports = mavlink_ctrl::MavlinkController::listSerialPorts();

// Connect. Baud 0 (or omitted) = the default of the link mode: 460800 (MAVLink) or 400000 (CRSF).
bool success = controller.connect("COM10", 460800);

// Disconnect gracefully
controller.disconnect();
```

### Callbacks

Register callbacks to react to state changes or raw MAVLink messages.

```cpp
controller.onStateChange([](mavlink_ctrl::ConnectionState state) {
    if (state == mavlink_ctrl::ConnectionState::LinkUp) {
        std::cout << "Target heartbeat received! Link is fully up." << std::endl;
    }
});

// Advanced: Tap into the raw MAVLink stream
controller.onMessage([](const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        // Handle custom unparsed message...
    }
});
```

### RC Control (Uplink)

Use `setChannel()` to send real-time control data via MAVLink `RC_CHANNELS_OVERRIDE`. These are instantly intercepted by the ELRS TX module and sent over the air at low latency.

```cpp
// Values are standard PWM microseconds (1000-2000µs). 
// 0 = release back to target, UINT16_MAX = ignore channel.
controller.setChannel(0, 1500); // Roll centered
controller.setChannel(1, 2000); // Pitch max
controller.setChannel(4, 2000); // ARM (Usually Ch5)
```

---

## 2. Telemetry (Downlink)

Telemetry is automatically parsed in the background. Call `getTelemetry()` to get a thread-safe snapshot of the current data.

```cpp
mavlink_ctrl::TelemetryData t = controller.getTelemetry();

// Example fields:
std::cout << "Battery: " << (t.battery_voltage_mv / 1000.0f) << "V\n";
std::cout << "Pitch: " << t.attitude_pitch << " rad\n";
std::cout << "RSSI: " << (int)t.radio_rssi << "\n";
```

### Supported MAVLink Messages

The `TelemetryData` struct automatically extracts and stores fields from the following MAVLink messages:

| Message Name | Message ID | Description |
|:---|:---|:---|
| `HEARTBEAT` | 0 | Target system state, type, and mode. |
| `SYS_STATUS` | 1 | Battery voltage/current, CPU load, drop rates. |
| `GPS_RAW_INT` | 24 | Standard GPS fix, lat/lon, altitude, satellites. |
| `ATTITUDE` | 30 | Roll, Pitch, Yaw in radians. |
| `GLOBAL_POSITION_INT` | 33 | Fused global position and velocities. |
| `SERVO_OUTPUT_RAW` | 36 | Real-time PWM output values driving the servos. |
| `RC_CHANNELS` | 65 | Current RC inputs seen by the target. |
| `VFR_HUD` | 74 | Airspeed, Groundspeed, Heading, Throttle, Climb. |
| `RADIO_STATUS` | 109 | ELRS RF link health (RSSI, SNR, Errors, Buffers). |
| `NAMED_VALUE_FLOAT` | 251 | Arbitrary floating point key-value pairs. |
| `NAMED_VALUE_INT` | 252 | Arbitrary integer key-value pairs. |
| `STATUSTEXT` | 253 | System console text messages (e.g., "Prearm failure"). |

### Custom Telemetry (`NAMED_VALUE_FLOAT`)

For custom robotics (like reading encoder angles from a Raspberry Pi), the `NAMED_VALUE_FLOAT` (Msg 251) is highly recommended. The controller automatically aggregates these into a `std::unordered_map`.

```cpp
// Check if a specific named value exists and print it
auto t = controller.getTelemetry();
if (t.named_values.contains("L1_Angle")) {
    std::cout << "Leg 1 Angle: " << t.named_values.at("L1_Angle") << " degrees\n";
}

// Alternatively, use the convenience accessor with a default value
float angle = controller.getNamedValue("L1_Angle", 0.0f);
```

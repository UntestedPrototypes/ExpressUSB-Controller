# ExpressUSB-Controller API Documentation

This library allows you to communicate natively with an ExpressLRS (ELRS) transmitter module over USB using the CRSF protocol.

## Core Classes

### crsf::CrsfController
The main class used to interface with the transmitter.

#### Initialization & Lifecycle
*   **CrsfController()**
    Creates a new controller instance.
*   **ool connect(const std::string& port_name, uint32_t baudrate)**
    Opens the serial connection to the transmitter.
    *   *Parameters:* 
        *   port_name: e.g. "COM10" (Windows) or "/dev/ttyUSB0" (Linux). The library automatically handles Windows high-COM port prefixes.
        *   audrate: Typically 115200 or 921600 (recommended).
    *   *Returns:* 	rue if opened successfully.
*   **oid disconnect()**
    Safely closes the serial port and shuts down background threads.

#### State & Callbacks
*   **ConnectionState getConnectionState() const**
    Returns the current link state:
    *   Disconnected: No module detected.
    *   TxConnected: Transmitter module connected, but no receiver link.
    *   LinkUp: Receiver is connected and telemetry is flowing.
*   **oid onParametersLoaded(std::function<void()> callback)**
    Registers a callback that fires automatically once all ELRS configuration parameters have been downloaded from the transmitter.

#### Telemetry
*   **TelemetryData getTelemetry() const**
    Returns a thread-safe snapshot of all current telemetry values. (See TelemetryData struct below).

#### RC Control
*   **oid setChannel(uint8_t index, uint16_t value_us)**
    Sets the RC channel value to be transmitted.
    *   *Parameters:*
        *   index: The channel index (0-15). Note: CH5 (index 4) is traditionally the ARM channel in ELRS.
        *   alue_us: PWM microsecond value (988 to 2012). Center is typically 1500.

#### Special Commands
*   **enum class ElrsAction**
    *   Bind, WiFiConnectivity, BleJoystick, VtxAdministrator, ResetVtx, EnterBootloader
*   **ool executeAction(ElrsAction action)**
    Triggers a built-in ELRS module action (like entering Bind mode).
*   **ool executeCommand(const std::string& command_name)**
    Triggers an ELRS command parameter by its string name.
*   **ool triggerBind()**
    Convenience method equivalent to executeAction(ElrsAction::Bind).

---

### crsf::TelemetryData
A struct containing all decoded telemetry. Note that not all fields will be populated unless the receiver has the appropriate sensors attached (e.g., GPS).

#### Standard Fields
*   std::string device_name (e.g. "ExpressLRS TX")
*   uint8_t uplink_lq: Link Quality (0-100%).
*   int8_t uplink_snr: Signal to Noise Ratio.
*   uint8_t uplink_rssi_1 / uplink_rssi_2: dBm values (as positive integers, multiply by -1).
*   uint8_t uplink_tx_power: Current transmit power index.
*   uint8_t rf_mode: Current packet rate / RF mode index.

#### Extended Sensors (ELRS 3.x / 4.x)
*   **Battery:** attery_voltage (100mV), attery_current (100mA), attery_capacity (mAh), attery_remaining (%)
*   **High-res Cells:** std::vector<uint16_t> cells_values (millivolt precision per cell)
*   **GPS:** gps_latitude, gps_longitude, gps_altitude, gps_groundspeed, gps_heading, gps_satellites
*   **Attitude:** ttitude_pitch, ttitude_roll, ttitude_yaw (radians * 10000)
*   **Vario:** ario_verticalspd (cm/s)
*   **Baro Altitude:** aro_altitude (m), aro_verticalspd (cm/s)
*   **Airspeed:** irspeed_speed (km/h)
*   **RPM:** std::vector<int32_t> rpm_values
*   **Temperature:** std::vector<int16_t> temp_values

#### Status & State
*   ool is_armed: Real-time arm state extracted from ELRS 4.0 channel status bytes.
*   uint16_t elrs_pkts_good, uint8_t elrs_pkts_bad: Protocol layer transmission stats.
*   std::string elrs_status_msg: Plaintext status strings like "Model Mismatch".

---

### crsf::Parameter
Represents an ExpressLRS configuration setting (e.g., "Packet Rate").

*   uint8_t index: Internal protocol ID.
*   std::string label: Display name.
*   uint8_t type: Data type (e.g., 9 = TEXT_SELECTION, 13 = COMMAND).
*   *For TEXT_SELECTION types:*
    *   std::vector<std::string> options: The list of valid options.
    *   uint8_t current_index: The currently selected option.

You can fetch all parameters by calling std::vector<Parameter> params = controller.getParameters(); after the onParametersLoaded callback has fired.

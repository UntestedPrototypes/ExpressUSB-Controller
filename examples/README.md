# ExpressUSB Examples

This directory contains example applications demonstrating how to use the `MavlinkController` library to interface with an ExpressLRS TX module, in Native MAVLink mode (default) or in CRSF mode (`--mode crsf`).

Both examples take `<COM_PORT> [BAUD] [--mode mavlink|crsf]`. Without a baud rate they use the default of the mode (460800 for MAVLink, 400000 for CRSF).

## Building the Examples

The examples are automatically built when you build the main library.

```powershell
# From the project root
cmake -B build/windows-debug -S .
cmake --build build/windows-debug
```

## Example 1: `basic_connection.exe`

A diagnostic tool that connects to the ELRS TX module, broadcasts GCS heartbeats, and prints incoming MAVLink telemetry at a 2-second interval.

**Usage:**
```powershell
.\build\windows-debug\expressusb_example.exe COM10 460800
.\build\windows-debug\expressusb_example.exe COM10 --mode crsf    # CRSF: link statistics, battery, status text
```

**Expected Output:**
```
Connecting to COM10 @ 460800 baud...
[STATE] Connected (waiting for remote heartbeat...)
Connected! Sending GCS heartbeats at 1Hz.
Press Ctrl+C to exit.

[STATE] Link Up! Remote system detected.
--- Telemetry Snapshot ---
  Remote: SysID=1 CompID=1 Type=2 Status=4
  Battery: 15.4V | 2.1A | 85%
  Attitude: R=2.1° P=-1.4° Y=184.2°
  Radio: RSSI=-65 RemRSSI=-62 TxBuf=100% RxErr=0
  Named Values:
    L1_Angle = 45.20
    L2_Angle = 30.15
```

## Example 2: `rc_control_example.exe`

An interactive command-line application that reads input from a standard USB Gamepad (XInput on Windows, `/dev/input/js0` on Linux) and transmits it as `RC_CHANNELS_OVERRIDE` MAVLink commands at 50Hz.

If no gamepad is detected, it falls back to a non-blocking keyboard interface using `WASD` / `IJKL`.

**Usage:**
```powershell
.\build\windows-debug\rc_control_example.exe COM10 460800
.\build\windows-debug\rc_control_example.exe COM10 --mode crsf    # sends CRSF RC frames at 250 Hz
```

**Expected Output:**
```
Connecting to COM10 @ 460800 baud...
[STATE] Connected (waiting for remote heartbeat...)
Connected!

=== RC Controller (MAVLink) ===
Gamepad: Auto-detected (XInput / /dev/input/js0)
Keyboard (Fallback):
  W/S : Pitch
  A/D : Roll
  I/K : Throttle
  J/L : Yaw
  SPACE : Toggle Arm
  Q : Quit

[RC Out] [GAMEPAD] R:1500 P:1500 T:1000 Y:1500 ARM:DISARMED | RSSI:-65
```

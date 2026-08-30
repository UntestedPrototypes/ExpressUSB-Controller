# ExpressUSB-Controller Examples

This folder contains example programs demonstrating how to use the `ExpressUSB` library to communicate with an ExpressLRS transmitter module over USB.

## How to Build

All examples are automatically built when you compile the main project. Using CMake or your IDE (like VS Code), build the project as normal. The compiled executables will be placed in your build directory (e.g., `build/windows-debug/`).

---

## 1. Basic Connection & Telemetry (`basic_connection.cpp`)

**What it does:**
This is the primary diagnostic tool. It connects to the transmitter, automatically downloads the configuration menu, and then continuously prints a real-time summary of the link state, battery, RC link quality, and any extended ELRS telemetry.

**How to run (Windows):**
```powershell
.\build\windows-debug\expressusb_example.exe COM10 921600
```
*(Replace `COM10` with the actual COM port of your module)*

**Expected Terminal Output:**
```text
Connecting to COM10 @ 921600 baud...
[STATE] TX Module Connected (no receiver)
Connected. Press Ctrl+C to exit.
Telemetry will print every 2 seconds.

=== Telemetry [23:12:06] ===
Connection: LINK UP
Device: ExpressLRS TX
Link Quality: 100%
RSSI: -55 dBm
RF Mode: 4
TX Power: 3
Battery: 12.6V  1.2A  95%
Cell Voltages: 4202mV 4198mV 4201mV 
Arm State: ARMED (Status byte: 0x1)
ELRS Status: 250 Good / 0 Bad (Flags: 0x8)

[INFO] All parameters loaded!
Parameters (12 total):
  [1] Packet Rate = 250Hz (3)
  [2] Telem Ratio = Std (0)
  ...
```

---

## 2. RC Control (`rc_control_example.cpp`)

**What it does:**
Demonstrates how to safely send joystick/RC data to the drone interactively. It features **native auto-detecting Gamepad support** (XInput on Windows or `/dev/input/js0` on Linux) so you can fly your drone using a real Xbox/PlayStation controller!

If no gamepad is connected, it seamlessly falls back to cross-platform keyboard controls (WASD). All inputs instantly snap to 1500 (center) or max/min values, with no software smoothing, simulating true snappy RC control.

**How to run (Windows):**
```powershell
.\build\windows-debug\rc_control_example.exe COM10 921600
```

**Expected Terminal Output:**
```text
Connecting to COM10 @ 921600 baud...
[STATE] TX Module Connected (no receiver)
Connected!

=== RC Controller ===
Gamepad: Auto-detected (XInput / /dev/input/js0)
Keyboard (Fallback):
  W/S : Pitch
  A/D : Roll
  I/K : Throttle
  J/L : Yaw
  SPACE : Toggle Arm
  Q : Quit

[RC Out] [GAMEPAD] R:1500 P:1500 T:1000 Y:1500 ARM:DISARMED
```
*(Pressing WASD will instantly animate the stick values, and they will spring back to 1500 when you let go!)*

---

## 3. Special Commands (`special_commands_example.cpp`)

**What it does:**
Demonstrates the correct asynchronous flow to trigger an action (like putting the module into Bind Mode). It connects, waits for the background thread to finish downloading the configuration parameters, and immediately fires off a Bind command.

**How to run (Windows):**
```powershell
.\build\windows-debug\special_commands_example.exe COM10 921600
```

**Expected Terminal Output:**
```text
Connecting to COM10 @ 921600 baud...
[STATE] TX Module Connected (no receiver)
Connected! Waiting for module parameters to load...

[SUCCESS] All parameters loaded!

=== Available Special Commands ===
1. VTX Administrator
2. Bind
3. WiFi Connectivity
4. BLE Joystick
5. Reset VTX
6. Enter Bootloader

Enter the number of the command to execute (0 to exit): 2
Executing 'Bind'...
Command sent successfully! Check your TX module.

Shutting down...
```

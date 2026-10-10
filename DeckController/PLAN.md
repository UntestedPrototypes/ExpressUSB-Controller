# DeckController: implementation plan (revision 2, for review)

A Steam Deck application that drives the PentaPod over ExpressLRS. It uses the ExpressUSB library in this repository to talk to an
ELRS TX module plugged into the Deck's USB-C port, in **CRSF mode** to start with (MAVLink later, same library API). The joysticks walk
the robot, the touchpads orient its body, the screen shows the robot's and the link's state. **Qt 6 Quick** UI, buildable **on the Deck
and on a Windows PC**.

Nothing is built yet. Open points are marked **[decide]**.

Decisions taken (review 1): Qt 6 Quick; body height as a rate; left touchpad X does nothing; hold A 2 s to arm, A again to disarm;
**B = emergency stop**; build on the Deck and on Windows; add as much robot telemetry as possible (including the posture).

---

## 1. Control scheme

```
        L2 ---------------------------------------------------------- R2  speed scale (analog)
        L1 ---------------------------------------------------------- R1
   +-------------------------------------------------------------------------+
   |  [View] page                                          [Menu] reset E-stop (hold 2 s) |
   |   ___                                                                        ___     |
   |  /   \  LEFT STICK                                         Y  [ - ]       /   \     |
   |  \___/  walk: x / y                               X gait       B E-STOP    \___/     |
   |                                                              A arm/disarm            |
   |  D-pad: up/down step height                                                 RIGHT    |
   |         left/right IK solver                                                STICK    |
   |  +------------+                 [ screen: dashboard ]           +------------+ x yaw |
   |  | LEFT PAD   |                                                 | RIGHT PAD  | y height (rate)
   |  | y: stance  |                                                 | x: roll    |       |
   |  | click:reset|                                                 | y: pitch   |       |
   |  +------------+                                                 | click:reset|       |
   +-------------------------------------------------------------------------+
     back grips:  L4 = record stance (hold while lifting the finger)    R4 = record tilt (hold while lifting)
                  L5, R5 = free
```

| Input | Action | Robot channel |
|---|---|---|
| **Left stick** | walking vector: up = forward (X), right = right (Y). Length = speed (capped at 1), dead zone + expo in the app. Released = stand still. | CH2 X, CH1 Y |
| **Right stick X** | yaw rate, right = clockwise | CH4 |
| **Right stick Y** | **body height as a rate**: up = rise (full stick = 50 mm/s, settable); released = the height stays | CH3 (the app sends the held setpoint as an absolute value) |
| **R2** (analog) | speed scale: released = 40 % of the robot's top speed, fully pressed = 100 % (both settable) **[decide]** | scales CH1 / CH2 |
| **Right touchpad** | **tilt**: while a finger is on it, its movement from where it landed sets roll (X) and pitch (Y); lifting the finger returns to the tilt trim (straight unless recorded) | CH6 roll, CH7 pitch |
| **R4** (back grip) held while lifting the finger from the right pad | **record** the tilt as the new trim: the body stays tilted, also while walking | |
| **Right touchpad click** | tilt trim back to straight | |
| **Left touchpad Y** | **stance**: up = wider, same "relative to where the finger landed" | CH11 |
| **L4** held while lifting from the left pad | record the stance | |
| **Left touchpad click** | stance back to its default | |
| Left touchpad X | nothing | |
| **A, hold 2 s** (while disarmed) | **arm**: a progress ring on the screen fills; at 2 s the arm channel goes on, the robot stands up (from Standby) | CH5 |
| **A, press** (while armed) | **disarm**: the robot stops and goes to Standby | CH5 |
| **B** | **EMERGENCY STOP**: all motors released at once, latched on the robot; the app also disarms. Works in every state, also while disarmed | CH12 (new E-stop channel, see 3.1) |
| **Menu, hold 2 s** (after an E-stop, disarmed) | reset the E-stop: the robot clears the latched failsafe; it stays disarmed **[decide]** | CH12 back to off |
| **X** | gait: wave / ripple | CH8 |
| **D-pad up / down** | step height +/- 5 mm | CH10 |
| **D-pad left / right** | IK solver: analytic / auto / numeric | CH9 |
| **View** | next dashboard page (main, channels, log, settings) | |
| Y, L1, R1, L2, L3, R3, L5, R5 | free | |

Safety rules in the app:
- The app starts **disarmed**, E-stop off. Every arm needs the 2 s hold; the robot itself also only arms on an off-to-on change.
- Controller disconnected, app minimised or loses focus, any input error: **disarm** at once (the robot goes to Standby).
- App closed or crashed: the TX module gets no more frames, the receiver goes to failsafe, the robot sees the link lost after 250 ms, holds,
  and goes to Standby after 3 s (to verify on the bench that the receiver stops sending channels).
- The walking vector, yaw and height rate are ignored while disarmed (so nothing is "waiting" in the sticks when you arm).
- Arming is refused by the app while the E-stop is active.

**Touchpad risk:** inside Steam, Steam Input normally turns the touchpads into a mouse. The app reads the controller with **SDL3**
(Qt has no gamepad module in Qt 6), whose Steam Deck driver reports touchpad positions and clicks. That probably needs Steam Input to
be disabled for the app (per-game controller setting) or the app to run outside Steam. Checked first, in milestone 1.

## 2. Channels (the robot's full resolution map, updated)

| CH | Value | Rate in 12ch Mixed |
|---|---|---|
| 1 | Y factor (+ = right) | every packet |
| 2 | X factor (+ = forward) | every packet |
| 3 | body height (absolute) | every packet |
| 4 | yaw rate (+ = clockwise) | every packet |
| 5 | arm | every packet (1 bit) |
| 6 | roll | half rate |
| 7 | pitch | half rate |
| 8 | gait | half rate |
| 9 | IK solver | half rate |
| 10 | step height | half rate |
| 11 | stance | half rate |
| 12 | **E-stop** (new; was Pi control, which becomes "not used" by default but stays mappable) | half rate |

- TX module settings (Lua script or web UI; the library cannot set them): link mode **CRSF**, switch mode **Full Resolution 12ch Mixed**,
  packet rate **100 Hz Full** or **333 Hz Full**, a telemetry ratio that leaves room for the robot's downlink (1:4 or 1:8, tuned on the bench).
- The library converts microseconds to CRSF with the radio scale (988 us = -100 %, 1500 = 0, 2012 = +100 %). The app sends **988 to 2012 us**
  for full travel.
- The E-stop on a half-rate channel adds at most one packet of delay (10 ms at 100 Hz).

## 3. Firmware changes this needs (PentaPod)

### 3.1 Emergency stop over the radio
- New function **`estop`** in the channel map (`RcMap`, default **CH12**; Pi control default "not used"). The PentaPod app's channel map
  table gets the row.
- Channel above +0.5 (as an off-to-on change seen with the link up): **failsafe with fault E-stop**, all motors released, exactly like the
  USB E-stop. Arming is ignored while the channel is on.
- **Reset**: the E-stop channel going from on to off, seen with the link up **and the radio disarmed**, clears the latched failsafe. **[decide]**
  whether this reset clears only an E-stop or any latched fault (crash, limit, ...). Recommendation: any fault, since the operator
  acknowledges it deliberately (Menu, 2 s) and the robot stays disarmed.
- A link loss never resets anything; an app that starts with the channel off does not reset (no on-to-off change was seen).
- Host tests for all of the above.

### 3.2 More telemetry over CRSF
The CRSF downlink is small (frames of up to 64 bytes, a few per second, shared with ELRS's own link statistics). Plan, using only frame
types ELRS forwards and the library already decodes:

| Frame | Content (new unless marked) | When |
|---|---|---|
| Flight mode (0x21), **status line v2** | state, phase, **posture**, fault, stop reason, fault servo, shut-down legs, readable text | on every change, else every 5 s |
| Flight mode (0x21), **detail line** (new, starts with `#`) | hottest servo and its temperature, battery voltage, servo supply, gait type, IK solver, step height, stance, body height, walking speed, sequence name and step | every 2 s |
| Attitude (0x1E) | the robot's **active body roll and pitch** (what the controller applies after slewing), yaw field = active yaw rate | 2 per second while active |
| Battery (0x08) | voltage = **battery pack** (PCB divider) if wired, else the servo supply (as now); remaining % estimated from the 3S voltage | every 2 s |

Status line v2 (fits in 59 characters):

```
<S><P><Q> F<f> W<w> S<ss> L<mm> <text>        e.g.  A2I F0 W0 S00 L00 Idle
```

`Q` is the new **posture** letter: `F` folded (stowed), `B` standby, `U` standing up, `I` **idle**, `L` lowering to Standby, `Q` sequence
running, `X` failsafe, `?` unknown pose. The robot decides it like the PentaPod app does: the controller phase while it drives, else the
measured joints compared with the folded and the Standby pose (15 degrees per joint), the sequence status while one runs.

Detail line (example): `#T08:61 B11.6 U7.4 GW QA H140 SH035 ST200 V045 SEQfold:2/3` (at most 59 characters) (servo 8 hottest at 61 C, battery 11.6 V, supply 7.4 V,
wave gait, analytic solver, height 140 mm, step height 35 mm, stance 200 mm, speed 45 mm/s, sequence fold at step 2 of 3). The app keeps
the status line and the detail line apart by the `#`.

Bandwidth: about 5 robot frames per 10 s for status and detail, plus 2 attitude frames per second. To be tuned on the bench (the order of
priority if the link is short: status line, battery, attitude, detail line).

### 3.3 The same over MAVLink (later, milestone 8)
`NAMED_VALUE_INT` for `POSTURE`, `BATT`, `TEMPSERVO`, `GAIT`, `SOLVER`, `SEQSTEP`; `ATTITUDE` with the active roll / pitch; the posture in
`HEARTBEAT.custom_mode`.

Not possible over the radio at a useful rate: all joint angles, all temperatures, the IK state. Those stay on the USB / PentaPod app.

## 4. What the dashboard shows

The Deck screen (1280 x 800, touch) is a dashboard; driving is done with the sticks and pads. Pages switched with **View** (or touch):

1. **Main**
   - **Status bar**: big **DISARMED / ARMING (ring) / ARMED / E-STOP** indicator; the robot's **posture** (Folded, Standby, Standing up, Idle,
     Lowering, Sequence, FAILSAFE) and state; link: uplink / downlink LQ, RSSI, packet rate, TX power (red when down); battery and servo
     supply with colours; **telemetry age** (red if no robot status for > 6 s); Deck battery.
   - **Fault banner** when there is a fault or a stop reason: the text, the servo and its leg.
   - **Robot view**: the five legs from above, shut-down legs red, the hottest servo marked; over it the **commanded** walking vector, yaw arc
     and stance ring, and an **attitude indicator** showing the commanded tilt and the robot's **reported** tilt side by side.
   - **Setpoints**: body height (commanded and reported), roll / pitch trims, stance, step height, gait, solver, speed scale, each with a
     reset (touch).
2. **Channels**: the 12 channel values sent (us and %), for checking the map against the PentaPod app.
3. **Log**: every status change with a time stamp (the last 200), so a short fault is not missed.
4. **Settings**: serial port, link mode (CRSF / MAVLink), the channel map (default = the robot's default map), dead zones, expo, rates,
   speed scale limits, inverts. Saved in the user's config folder (`~/.config/DeckController/`, `%APPDATA%\DeckController\`).

## 5. Architecture

```
SDL3 gamepad (sticks, touchpads, buttons, triggers)        Qt 6 Quick UI (QML pages)
      |                                                            ^
 InputReader (QObject, polls SDL at 250 Hz on its own thread)       |  properties / signals
      |                                                            |
 ControlModel (dead zones, expo, rates, trims, record, height setpoint, arm / E-stop state machine)    pure C++, unit tested
      |
 ChannelMapper (the robot's channel map, 988-2012 us)  ->  ExpressUSB MavlinkController::setChannel()  ->  TX module (CRSF, 250 Hz)
                                                                                                                 |
 RobotState  <-  TelemetryParser (status line v2, detail line, attitude, battery; MAVLink named values later)  <-+
      |
 DashboardModel (QObject exposed to QML)
```

- **Folder** `ExpressUSB-Controller/DeckController/`: `CMakeLists.txt`, `src/` (C++), `qml/`, `tests/`, `packaging/`. Added to the top-level
  build with an option `EXPRESSUSB_BUILD_DECK` (off by default, so the library still builds without Qt / SDL).
- **Shared code with the robot**: the tests compile the PentaPod firmware's `rc_mapper.cpp`, `crsf_parser.cpp` and `crsf_telemetry.cpp`
  (read only, as the library tests already do), so the app and the robot agree on channels and telemetry text.
- **Threads**: SDL input on its own thread (250 Hz) feeding the ControlModel; the library sends RC on its own thread at 250 Hz; telemetry
  read every UI frame (60 Hz).

### Building on both machines

| | Windows PC | Steam Deck |
|---|---|---|
| Toolchain | Qt 6.12 MinGW kit under `C:\Qt` (the one the PentaPod app uses), CMake, Ninja | **distrobox** with an Arch container (SteamOS's root is read-only): `qt6-base qt6-declarative sdl3 cmake ninja gcc` |
| SDL3 | built from source by CMake (FetchContent) if not installed | the Arch package |
| Build | `build.bat` (like the PentaPod app): configure, build, test, `install` to `dist\` with `windeployqt` | `build.sh`: same steps |
| Run | `dist\bin\DeckController.exe` | in Desktop Mode from the container; for **Game Mode** an **AppImage** (linuxdeploy + Qt plugin) added as a non-Steam game |
| Serial port | the TX module's COM port | `/dev/ttyUSB0` / `ttyACM0`; the `deck` user must be in the `uucp` group (one-time `sudo usermod -aG uucp deck` after setting a password) |

Windows is also where most development and the tests run; the Deck only needs the controller readout (SDL works the same with a normal
gamepad on Windows, minus the Deck touchpads; on Windows the touchpads are simulated with the mouse on the dashboard for testing).

### Tests (CTest, no hardware)
- ControlModel: dead zones, expo, height rate and limits, tilt / stance relative touch, record / reset, arm hold 2 s, disarm, E-stop and its
  reset gesture, disarm on controller loss.
- ChannelMapper **through the robot's own RcMapper**: the values the app sends come out on the robot as the intended speed, direction, yaw,
  height, roll, pitch, stance, arm and E-stop.
- TelemetryParser against the firmware's own status / detail line builder for many states, and the attitude / battery frames.
- End to end over the library's loopback transport (`setTransport`).
- Firmware host tests for 3.1 and 3.2.

## 6. Milestones

| # | Where | Content | Done when |
|---|---|---|---|
| 1 | Deck app | folder, CMake for both machines, Qt Quick window, SDL3 input incl. **touchpads on the Deck**, connect to the TX module in CRSF, link statistics on screen | the Deck shows stick and touchpad values and the LQ of a live link; Windows builds too |
| 2 | Deck app | ControlModel, ChannelMapper, channels page, tests incl. the robot's RcMapper | tests pass on both machines |
| 3 | Firmware + PentaPod app | E-stop channel and reset (3.1), channel map row, tests | B on a test transmitter releases all motors; reset works |
| 4 | Firmware | telemetry v2: posture, status line v2, detail line, attitude, battery (3.2), tests | the new frames decode in the library tests |
| 5 | Deck app | TelemetryParser, status bar, fault banner, log, robot view, attitude indicator | a provoked leg fault and the postures show on the Deck |
| 6 | Deck app | touchpad tilt / stance, record and reset, height rate, arm hold / disarm, E-stop and reset, speed scale | bench test: walk, tilt while walking, record, disarm, E-stop |
| 7 | Deck app | settings page and file; Deck packaging (AppImage, Game Mode, Steam Input settings, serial permission); bench checklist | runs from Game Mode with the TX module plugged in |
| 8 | both | MAVLink mode: library MAVLink link, the 3.3 telemetry | same dashboard over MAVLink |

## 7. Decisions (review 2) and status

1. E-stop reset clears **any** latched fault. 2. Speed scale on R2 stays. 3. 100 Hz Full, telemetry 1:4. 4. Pi control moves off CH12.
Groundwork (core, app shell, build on Windows) is done, see [README.md](README.md). Next: milestone 1 on the Deck, milestone 3 firmware.

## 7a. Original questions

1. **E-stop reset** (Menu held 2 s): should it clear only an E-stop, or any latched fault (crash, limit, servo, ...)? I recommend any fault.
2. **Speed scale on R2** (40 % released, 100 % pressed): keep, or always full speed?
3. **Telemetry ratio and rate**: start with 100 Hz Full, telemetry 1:4? (Longer range than 333 Hz, more downlink time.)
4. **Pi control** moves off CH12 to "not used" by default. Fine?

# DeckController

Steam Deck / PC application that drives the PentaPod over an ExpressLRS TX module, using the ExpressUSB library in the folder above.
Plan, control scheme and milestones: [PLAN.md](PLAN.md).

## Status (groundwork, milestones 1 and 2 partly)

| Done | Not yet |
|---|---|
| Qt 6 Quick window with status bar, stick / touchpad views, channels page, settings page (port, CRSF / MAVLink, connect) | Robot telemetry decoding (posture, detail line), log page |
| SDL3 reader: sticks, R2, buttons, back grips, touchpads (two-pad Deck layout assumed) | **Verify on the Deck**: touchpad numbering and click, Steam Input settings |
| `ControlModel`: dead zone / expo, speed scale, height as a rate, relative-touch tilt and stance with record and reset, arm hold 2 s, disarm, E-stop and its reset, disarm on controller loss | Settings file, saved channel map editor |
| `ChannelMapper`: the robot's full resolution map (CH12 = E-stop), remappable and invertible, 988..2012 us | Firmware: E-stop channel, telemetry v2 (PLAN 3.1, 3.2) |
| 250 Hz link loop, disarm when the window loses focus | Deck packaging (AppImage) |
| Core self-tests (`core_tests`) | Tests through the robot's own `RcMapper`, loopback end-to-end |

Until the firmware has the E-stop channel (milestone 3), the robot ignores CH12 (or reads it as Pi control, the old default): the **B button
still stops the app's own output** (disarms), but the radio E-stop does not release the motors yet.

## Build

| | Windows | Steam Deck (Arch distrobox) / Linux |
|---|---|---|
| Needs | Qt 6.12 MinGW kit under `C:\Qt` (the PentaPod app's), internet for the first build (asio, MAVLink, SDL3 are fetched) | `pacman -S qt6-base qt6-declarative qt6-svg sdl3 cmake ninja gcc` |
| Build + test | `build.bat test` | `./build.sh test` |
| Core only | `build.bat core` | `./build.sh core` |
| Install | `build.bat install` -> `dist\bin\DeckController.exe` | |

Or from the top-level project: `cmake -DEXPRESSUSB_BUILD_DECK=ON`.

## Layout

```
src/core/   ControlModel, ChannelMapper, InputState   pure C++, unit tested
src/app/    Backend (timer, link, QML properties), InputReader (SDL3), main.cpp, qml/
tests/      core_tests.cpp
```

# DeckController

Steam Deck / PC application that drives the PentaPod over an ExpressLRS TX module, using the ExpressUSB library in the folder above.
Plan, control scheme and milestones: [PLAN.md](PLAN.md).

## Status

| Done | Not yet |
|---|---|
| Qt 6 Quick dashboard: status bar (arm / E-stop, link, **robot posture**, battery, hottest servo), fault banner, robot values, **log page**, channels page, settings page | Robot view with legs, attitude indicator, settings file, channel-map editor |
| SDL3 reader: sticks, R2, buttons, back grips, touchpads (two-pad Deck layout assumed) | **Verify on the Deck**: touchpad numbering and click, Steam Input settings |
| `ControlModel`, `ChannelMapper` (CH12 = E-stop), `TelemetryParser` (status line, detail line, attitude) | MAVLink telemetry (milestone 8), Deck packaging (AppImage) |
| Firmware (PentaPod, protocol 9): **E-stop channel + reset**, posture, status line v2, detail line, attitude frame, battery pack voltage | Nothing is tested on the real robot, radio or Deck yet |
| Tests: `core_tests` and `interop_tests` (the app's channels through the robot's own `RcMapper`, the robot's own telemetry through the parser) | End-to-end test over the library's loopback transport |

The robot has to run the firmware of PentaPod protocol revision 9 (E-stop channel, telemetry v2); on older firmware CH12 does nothing and
the dashboard shows "Robot: no data".

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
tests/      core_tests.cpp, interop_tests.cpp (needs ../../PentaPod/src and one firmware build for robot.pb.h)
```

#!/usr/bin/env bash
# Build DeckController on Linux / the Steam Deck (inside the Arch distrobox: pacman -S qt6-base qt6-declarative sdl3 cmake ninja gcc)
#   ./build.sh          configure + build
#   ./build.sh test     build, then run the tests
#   ./build.sh core     only the core and its tests (no Qt, no SDL)
set -euo pipefail
cd "$(dirname "$0")"
APP=ON; [ "${1:-}" = core ] && APP=OFF
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DDECK_BUILD_APP=$APP
cmake --build build
if [ "${1:-}" = test ] || [ "${1:-}" = core ]; then ctest --test-dir build --output-on-failure; fi
echo "Done: $(pwd)/build/src/app/DeckController"

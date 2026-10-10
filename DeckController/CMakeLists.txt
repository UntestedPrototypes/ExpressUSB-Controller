cmake_minimum_required(VERSION 3.21)
project(DeckController VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Standalone build (build.bat / build.sh): pull in the ExpressUSB library next to this folder.
# Built from the top-level project (-DEXPRESSUSB_BUILD_DECK=ON) the target already exists.
if(NOT TARGET ExpressUSB)
    set(EXPRESSUSB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(EXPRESSUSB_BUILD_DECK OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_BINARY_DIR}/expressusb")
endif()

# ---- core: control scheme and channel mapping. No Qt, no SDL.
add_library(deck_core STATIC
    src/core/ControlModel.cpp
    src/core/ChannelMapper.cpp
    src/core/TelemetryParser.cpp)
target_include_directories(deck_core PUBLIC src)

enable_testing()
add_executable(core_tests tests/core_tests.cpp)
target_link_libraries(core_tests PRIVATE deck_core)
add_test(NAME core_tests COMMAND core_tests)
if(MINGW)
    target_link_options(core_tests PRIVATE -static)
endif()

# Interoperability with the robot's own code (PentaPod firmware sources, read only), if the project sits next to this one:
# the app's channels through the robot's RcMapper, the robot's telemetry through the app's parser
set(PENTAPOD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../PentaPod/src" CACHE PATH "PentaPod firmware sources (optional)")
if(EXISTS "${PENTAPOD_SRC}/control/rc_mapper.cpp")
    # generated protobuf header needed by rc_map.h: present after one firmware build (pio run)
    if(EXISTS "${PENTAPOD_SRC}/robot.pb.h")
        add_executable(interop_tests tests/interop_tests.cpp
            "${PENTAPOD_SRC}/control/rc_mapper.cpp"
            "${PENTAPOD_SRC}/drivers/crsf_parser.cpp"
            "${PENTAPOD_SRC}/drivers/crsf_telemetry.cpp"
            "${PENTAPOD_SRC}/drivers/mavlink_telemetry.cpp"
            "${PENTAPOD_SRC}/drivers/mavlink_min.cpp")
        target_include_directories(interop_tests PRIVATE "${PENTAPOD_SRC}" "${PENTAPOD_SRC}/../.pio/libdeps/native/Nanopb")
        target_link_libraries(interop_tests PRIVATE deck_core)
        set_target_properties(interop_tests PROPERTIES CXX_STANDARD 17)   # the generated robot.pb.h uses 'requires' as a name
        add_test(NAME interop_tests COMMAND interop_tests)
        if(MINGW)
            target_link_options(interop_tests PRIVATE -static)
        endif()
    endif()
endif()

# ---- the application: Qt 6 Quick + SDL3 (gamepad and Steam Deck touchpads)
option(DECK_BUILD_APP "Build the Qt Quick application" ON)
if(DECK_BUILD_APP)
    add_subdirectory(src/app)
endif()

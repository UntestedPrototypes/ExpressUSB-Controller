cmake_minimum_required(VERSION 3.16)
project(ExpressUSB_Controller VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch standalone Asio (no Boost required)
include(FetchContent)
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-28-0
)
FetchContent_MakeAvailable(asio)

if(NOT TARGET asio)
    add_library(asio INTERFACE)
    target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
endif()

# Fetch MAVLink C library v2 (header-only)
FetchContent_Declare(
    mavlink_c_lib
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(mavlink_c_lib)

add_library(mavlink INTERFACE)
target_include_directories(mavlink INTERFACE ${mavlink_c_lib_SOURCE_DIR})

# --- Library ---
add_library(ExpressUSB STATIC
    src/mavlink_serial_port.cpp
    src/mavlink_handler.cpp
    src/crsf_handler.cpp
    src/mavlink_controller.cpp
)

target_include_directories(ExpressUSB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ExpressUSB PUBLIC asio mavlink)
target_compile_definitions(ExpressUSB PUBLIC ASIO_STANDALONE)

# Platform-specific link libraries
if(WIN32)
    target_link_libraries(ExpressUSB PRIVATE ws2_32 advapi32)
    target_compile_definitions(ExpressUSB PUBLIC _WIN32_WINNT=0x0601)
endif()

# --- Examples ---
add_executable(expressusb_example examples/basic_connection.cpp)
target_link_libraries(expressusb_example PRIVATE ExpressUSB)

add_executable(rc_control_example examples/rc_control_example.cpp)
target_link_libraries(rc_control_example PRIVATE ExpressUSB)

if(WIN32)
    # Required for Windows Gamepad support (XInput)
    target_link_libraries(rc_control_example PRIVATE xinput)
endif()

if(MINGW)
    target_link_options(expressusb_example PRIVATE -static)
    target_link_options(rc_control_example PRIVATE -static)
endif()

# --- Self-tests (no hardware needed): cmake -B build/x -S . && cmake --build build/x && ctest --test-dir build/x
option(EXPRESSUSB_BUILD_TESTS "Build the self-tests" ON)
if(EXPRESSUSB_BUILD_TESTS)
    enable_testing()
    add_executable(controller_tests tests/controller_tests.cpp)
    target_link_libraries(controller_tests PRIVATE ExpressUSB)
    # Optional interoperability check against the PentaPod robot's own CRSF parser (read only), if the
    # project sits next to this one
    set(PENTAPOD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../PentaPod/src" CACHE PATH "PentaPod firmware sources (optional)")
    if(EXISTS "${PENTAPOD_SRC}/drivers/crsf_parser.cpp")
        target_sources(controller_tests PRIVATE "${PENTAPOD_SRC}/drivers/crsf_parser.cpp")
        target_include_directories(controller_tests PRIVATE "${PENTAPOD_SRC}")
        target_compile_definitions(controller_tests PRIVATE EXPRESSUSB_TEST_PENTAPOD_INTEROP=1)
    endif()
    if(MINGW)
        target_link_options(controller_tests PRIVATE -static)
    endif()
    add_test(NAME controller_tests COMMAND controller_tests)
endif()

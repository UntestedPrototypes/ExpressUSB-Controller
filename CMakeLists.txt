cmake_minimum_required(VERSION 3.16)
project(ExpressUSB_Controller VERSION 1.0.0 LANGUAGES CXX)

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

# --- Library ---
add_library(ExpressUSB STATIC
    src/crc8.cpp
    src/frame_builder.cpp
    src/frame_reader.cpp
    src/frame_parser.cpp
    src/parameters.cpp
    src/serial_port.cpp
    src/crsf_controller.cpp
)

target_include_directories(ExpressUSB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ExpressUSB PUBLIC asio)
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

add_executable(special_commands_example examples/special_commands_example.cpp)
target_link_libraries(special_commands_example PRIVATE ExpressUSB)

if(MINGW)
    target_link_options(expressusb_example PRIVATE -static)
    target_link_options(rc_control_example PRIVATE -static)
    target_link_options(special_commands_example PRIVATE -static)
endif()

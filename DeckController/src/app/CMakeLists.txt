find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Quick QuickControls2)
qt_standard_project_setup()

# SDL3: an installed one (the Deck's Arch container: pacman -S sdl3), else built from source
find_package(SDL3 CONFIG QUIET)
if(NOT SDL3_FOUND)
    message(STATUS "SDL3 not installed: fetching and building it")
    include(FetchContent)
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    set(SDL_TESTS OFF CACHE BOOL "" FORCE)
    set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.2.10
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(SDL3)
    set(DECK_SDL_TARGET SDL3::SDL3-static)
else()
    set(DECK_SDL_TARGET SDL3::SDL3)
endif()

qt_add_executable(DeckController
    main.cpp
    InputReader.cpp
    InputReader.h
    Backend.cpp
    Backend.h)

qt_add_qml_module(DeckController
    URI DeckApp
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/StatusPill.qml
        qml/StickView.qml
        qml/PadView.qml)

target_link_libraries(DeckController PRIVATE
    Qt6::Core Qt6::Gui Qt6::Quick Qt6::QuickControls2
    deck_core ExpressUSB ${DECK_SDL_TARGET})
set_target_properties(DeckController PROPERTIES WIN32_EXECUTABLE ON)
if(MINGW)
    target_link_options(DeckController PRIVATE -static-libgcc -static-libstdc++)
endif()

install(TARGETS DeckController RUNTIME DESTINATION bin)
if(WIN32)
    qt_generate_deploy_qml_app_script(TARGET DeckController OUTPUT_SCRIPT deploy_script NO_UNSUPPORTED_PLATFORM_ERROR)
    install(SCRIPT ${deploy_script})
endif()

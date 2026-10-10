@echo off
rem Build (and optionally test / install) DeckController with the Qt kit under C:\Qt.
rem
rem   build.bat            configure + build            -> build\src\app\DeckController.exe
rem   build.bat test       build, then run the tests
rem   build.bat core       only the core (control model + tests); needs no Qt and no SDL
rem   build.bat install    build, then install into dist\  (dist\bin\DeckController.exe + the Qt DLLs)
rem
rem Override the kit with environment variables if yours differs:
rem   QT_ROOT (default C:\Qt)   QT_VERSION (default 6.12.0)   QT_KIT (default mingw_64)
setlocal
cd /d "%~dp0"

if "%QT_ROOT%"=="" set "QT_ROOT=C:\Qt"
if "%QT_VERSION%"=="" set "QT_VERSION=6.12.0"
if "%QT_KIT%"=="" set "QT_KIT=mingw_64"
set "QT_DIR=%QT_ROOT%\%QT_VERSION%\%QT_KIT%"
if not exist "%QT_DIR%\bin\Qt6Core.dll" (
    echo Qt kit not found: %QT_DIR%
    echo Set QT_ROOT / QT_VERSION / QT_KIT to match your installation.
    exit /b 1
)

set "TOOLS=%QT_ROOT%\Tools\CMake_64\bin;%QT_ROOT%\Tools\Ninja"
if /i "%QT_KIT:~0,5%"=="mingw" (
    for /d %%D in ("%QT_ROOT%\Tools\mingw*_64") do set "TOOLS=%%D\bin;%TOOLS%"
)
set "PATH=%TOOLS%;%QT_DIR%\bin;%PATH%"

set "APP=ON"
if /i "%1"=="core" set "APP=OFF"

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="%QT_DIR%" -DDECK_BUILD_APP=%APP% || exit /b 1
cmake --build build || exit /b 1

if /i "%1"=="test" ctest --test-dir build --output-on-failure || exit /b 1
if /i "%1"=="core" ctest --test-dir build --output-on-failure || exit /b 1
if /i "%1"=="install" (
    if exist dist rmdir /s /q dist
    cmake --install build --prefix dist || exit /b 1
    echo Installed: %~dp0dist\bin\DeckController.exe
)
echo.
echo Done.
exit /b 0

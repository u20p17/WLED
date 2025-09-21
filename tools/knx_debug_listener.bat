@echo off
REM KNX UDP Debug Listener for Windows
REM This script starts the Python UDP listener to capture debug messages from WLED ESP32

echo KNX UDP Debug Listener
echo =====================
echo.
echo This will listen for UDP debug messages from your WLED ESP32 on port 5140.
echo Make sure your ESP32 is on the same network as this PC.
echo.
echo Press Ctrl+C to stop the listener.
echo.

python "%~dp0knx_udp_debug_listener.py" %*

if errorlevel 1 (
    echo.
    echo ERROR: Python not found or script failed.
    echo Please make sure Python 3 is installed and in your PATH.
    echo.
    pause
)
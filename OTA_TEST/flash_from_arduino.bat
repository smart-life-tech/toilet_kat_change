@echo off
REM Flash script using Arduino IDE compiled binary
REM Usage: flash_from_arduino.bat COM3 path\to\compiled\binary.bin
REM
REM To find the compiled binary:
REM 1. In Arduino IDE, go to File -> Preferences
REM 2. Check "Show verbose output during: compilation"
REM 3. Compile your sketch
REM 4. Look for a line like: "C:\Users\...\AppData\Local\Temp\arduino\sketches\...\OTA_TEST.ino.bin"
REM 5. Copy that path and use it as the second argument

if "%1"=="" (
    echo Usage: flash_from_arduino.bat COMx path\to\binary.bin
    echo Example: flash_from_arduino.bat COM3 "C:\Users\...\OTA_TEST.ino.bin"
    echo.
    echo To find the binary path:
    echo 1. Arduino IDE: File -^> Preferences
    echo 2. Enable "Show verbose output during: compilation"
    echo 3. Compile and look for the .bin file path
    exit /b 1
)

if "%2"=="" (
    echo Error: Missing binary file path
    echo Usage: flash_from_arduino.bat COMx path\to\binary.bin
    exit /b 1
)

set PORT=%1
set BINARY=%2
set CHIP=esp32s3

REM Normalize path (convert forward slashes to backslashes for Windows)
set BINARY=%BINARY:/=\%

if not exist "%BINARY%" (
    echo Error: Binary file not found: %BINARY%
    exit /b 1
)

echo Flashing to factory partition (0x20000)...
echo Port: %PORT%
echo Binary: %BINARY%
echo.

REM Try to find esptool in common locations
set ESPTOOL=
where esptool.py >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set ESPTOOL=esptool.py
) else (
    REM Try Arduino IDE's esptool (common locations)
    if exist "%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\*\esptool.py" (
        for %%F in ("%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\*\esptool.py") do set ESPTOOL=%%F
    ) else if exist "%USERPROFILE%\.platformio\penv\Scripts\esptool.py" (
        set ESPTOOL=%USERPROFILE%\.platformio\penv\Scripts\esptool.py
    ) else (
        echo Trying python -m esptool...
        python -m esptool --chip %CHIP% --port %PORT% --baud 921600 write_flash 0x20000 "%BINARY%"
        goto :check_result
    )
)

REM Flash the binary to factory partition
"%ESPTOOL%" --chip %CHIP% --port %PORT% --baud 921600 write_flash 0x20000 "%BINARY%"

:check_result

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Success! Firmware flashed to factory partition.
    echo Device should now boot correctly.
) else (
    echo.
    echo Error: Flash failed. Check the port and try again.
)


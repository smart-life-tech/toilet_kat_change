@echo off
REM Flash script for OTA_TEST with proper partition table
REM Usage: flash_ota.bat COM3 (replace COM3 with your port)

if "%1"=="" (
    echo Usage: flash_ota.bat COMx
    echo Example: flash_ota.bat COM3
    exit /b 1
)

set PORT=%1
set CHIP=esp32s3
set BUILD_DIR=build\esp32.esp32.esp32s3usbotg

echo Erasing flash...
esptool.py --chip %CHIP% --port %PORT% erase_flash

echo.
echo Flashing bootloader...
esptool.py --chip %CHIP% --port %PORT% --baud 921600 write_flash 0x0 %BUILD_DIR%\OTA_TEST.ino.bootloader.bin

echo.
echo Flashing partition table...
esptool.py --chip %CHIP% --port %PORT% --baud 921600 write_flash 0x8000 %BUILD_DIR%\OTA_TEST.ino.partitions.bin

echo.
echo Flashing app to factory partition (0x20000)...
esptool.py --chip %CHIP% --port %PORT% --baud 921600 write_flash 0x20000 %BUILD_DIR%\OTA_TEST.ino.bin

echo.
echo Done! Device should now boot from factory partition.


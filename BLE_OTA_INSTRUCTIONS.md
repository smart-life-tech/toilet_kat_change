BLE OTA Instructions

This project already includes a BLE-based OTA implementation in `src/main.cpp` and a client script `ota_update.py` that can push a compiled firmware image to the device over BLE.

Quick summary
- The device advertises a main service and dynamically adds an OTA service when OTA mode is enabled (hold both hardware buttons for the configured trigger sequence).
- `ota_update.py` (requires `bleak`) implements the client side: it scans for the device, prepares the device for OTA, streams the firmware in chunks, sends an MD5, and finalizes the update.

Prerequisites
- Windows (PowerShell) or Linux/macOS with Python 3.8+
- `pip install bleak`
- PlatformIO (for building) — you can use the VS Code PlatformIO extension or the CLI `platformio` command.

Build firmware (PlatformIO)
Open PowerShell in the project root and run:

```powershell
# Build the project (PlatformIO CLI)
.pio\penv\Scripts\platformio.exe run

# The firmware binary will be under .pio\build\<env>\firmware.bin
# Typical path for default env (esp32dev):
$firmware = ".pio\build\esp32dev\firmware.bin"
Write-Host "Firmware path: $firmware"
```

If you use the PlatformIO extension, simply click Build; the firmware path is shown in the build output.

Flash over BLE using the provided Python script
1. Install dependencies (in PowerShell):

```powershell
python -m pip install --user bleak
```

2. Run the OTA script. The script can scan for the device name `ESP32 Toilet` or accept a BLE address.

Example (scan and update):

```powershell
python ota_update.py ".pio\build\esp32dev\firmware.bin"
```

Example (provide address explicitly):

```powershell
python ota_update.py ".pio\build\esp32dev\firmware.bin" --address "AA:BB:CC:DD:EE:FF"
```

Tips & troubleshooting
- Make sure the device is in OTA mode (hold both configured buttons for the OTA trigger sequence, see `src/main.cpp` behavior for details). The device will open an OTA window (default 1 minute) and advertise the OTA service UUID `5636340f-afc7-47b1-b0a8-15bcb9d7d29a6`.
- If the script fails to connect, run the scanner-only flow:

```powershell
# Run the scanner in Python REPL or temporary script to confirm device name/address
python - <<'PY'
from bleak import BleakScanner
import asyncio

a = asyncio.get_event_loop().run_until_complete(BleakScanner.discover(timeout=5.0))
for d in a:
    print(d)
PY
```

- BLE MTU and chunk size: the script uses 400 bytes per chunk by default; if you encounter write failures lower this to ~200.
- If the firmware is larger than the OTA partition the device may reject the update — the script prints firmware size and warns.


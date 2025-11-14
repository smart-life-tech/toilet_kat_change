# Arduino IDE Setup for OTA_TEST

## Partition Table Configuration

The `partitions.csv` file now includes:
- **factory** partition (0x20000) - Arduino IDE will flash here automatically
- **ota_0** partition (0x160000) - For OTA updates
- **ota_1** partition (0x2A0000) - For OTA updates (backup)

## Arduino IDE Configuration Steps

### Method 1: Use Custom Partition Scheme (Recommended)

1. **Copy the partition file:**
   - Copy `partitions.csv` to your Arduino sketch folder (same folder as `OTA_TEST.ino`)

2. **In Arduino IDE:**
   - Go to **Tools → Partition Scheme → Custom Partition Table**
   - Set **Custom Partition CSV** to `partitions.csv` (or browse to the file)

3. **Select your board:**
   - **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module** (or your specific board)
   - Set **Flash Size** to **4MB** (or larger if you have more)

4. **Upload:**
   - Click **Upload** - Arduino IDE will automatically:
     - Flash the partition table
     - Flash your code to the factory partition
     - The device will boot from factory partition

### Method 2: Use Built-in OTA Partition Scheme

If Method 1 doesn't work, try:

1. **Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)**
   - This gives you a factory partition Arduino IDE can use
   - But you'll need to manually configure OTA partitions

### After First Upload

Once uploaded, the device will:
- Boot from the **factory** partition
- OTA updates will go to **ota_0** or **ota_1**
- After an OTA update, the device will boot from the OTA partition

### Troubleshooting

If you get "No bootable app partitions":
1. **Erase flash first:**
   - Tools → Erase Flash → All Flash Contents
   - Then upload again

2. **Check partition scheme:**
   - Make sure "Custom Partition Table" is selected
   - Verify the CSV file path is correct

3. **Manual flash (if needed):**
   - Use the `flash_ota.bat` script with esptool.py


#!/usr/bin/env python3
"""
ESP32 Toilet System OTA Update Script
Updates firmware over Bluetooth Low Energy (BLE)

Usage:
    python ota_update.py <firmware.bin>                    # Update firmware (auto-scan for device)
    python ota_update.py <firmware.bin> --address XX:XX:XX:XX:XX:XX  # Update with specific address
    python ota_update.py <firmware.bin> --check-version  # Only check version, don't update

Requirements:
    pip install bleak

Example:
    python ota_update.py build/toilet_kat_change.bin
"""

import asyncio
import hashlib
import sys
import argparse
from bleak import BleakClient, BleakScanner
from typing import Optional

# ESP32 BLE Configuration (from toilet_kat_change.ino)
UPDATE_SERVICE_UUID = "5636340f-afc7-47b1-b0a8-15bcb9d7d29a6"
UPDATE_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea3"
VERSION_CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea2"
DEVICE_NAME = "ESP32 Toilet"
CHARACTERISTIC_UUID = "c327b077-560f-46a1-8f35-b4ab0332fea0"

# BLE MTU is typically 20-512 bytes, we'll use 400 bytes per chunk for safety
CHUNK_SIZE = 400


class OTAUpdater:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.update_progress = 0
        self.device_progress = 0
        
    def notification_handler(self, sender, data):
        """Handle notifications from device"""
        try:
            message = data.decode('utf-8')
            if "UPDATE_PROGRESS:" in message:
                # Extract progress percentage
                progress_str = message.split(":")[1]
                self.device_progress = int(progress_str)
                print(f"Device progress: {self.device_progress}%")
            elif "UPDATE_" in message:
                print(f"Device status: {message}")
        except Exception as e:
            print(f"Error handling notification: {e}")
        
    async def scan_for_device(self) -> Optional[str]:
        """Scan for ESP32 Toilet device"""
        print("Scanning for ESP32 Toilet device...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            if device.name == DEVICE_NAME:
                print(f"Found device: {device.name} at {device.address}")
                return device.address
        
        print("ESP32 Toilet device not found!")
        return None
    
    async def connect(self, address: str) -> bool:
        """Connect to ESP32 device"""
        try:
            self.client = BleakClient(address)
            await self.client.connect()
            # store address for potential reconnects
            self.address = address
            self.connected = True
            print(f"Connected to {DEVICE_NAME} at {address}")
            
            # Enable notifications for version characteristic to receive progress updates
            try:
                await self.client.start_notify(VERSION_CHARACTERISTIC_UUID, self.notification_handler)
            except Exception as e:
                print(f"Warning: Could not enable notifications: {e}")
            
            # Wait for connection to stabilize
            await asyncio.sleep(1.0)
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from ESP32 device"""
        if self.client and self.connected:
            try:
                # Stop notifications
                await self.client.stop_notify(VERSION_CHARACTERISTIC_UUID)
            except:
                pass
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from ESP32")
    
    async def check_version(self) -> Optional[str]:
        """Check current firmware version"""
        if not self.connected:
            print("Not connected to device")
            return None
        
        try:
            # Send CHECK_VERSION command
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"CHECK_VERSION")
            await asyncio.sleep(0.5)
            
            # Read version from version characteristic
            version_data = await self.client.read_gatt_char(VERSION_CHARACTERISTIC_UUID)
            version_str = version_data.decode('utf-8')
            print(f"Current firmware version: {version_str}")
            return version_str
        except Exception as e:
            print(f"Failed to check version: {e}")
            return None
    
    async def prepare_update(self) -> bool:
        """Prepare device for OTA update"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        try:
            print("Preparing device for update...")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"PREPARE_UPDATE")
            await asyncio.sleep(1.0)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_PREPARED" in response_str:
                print("Device prepared for update")
                return True
            elif "UPDATE_BLOCKED" in response_str:
                print("ERROR: Update preparation blocked (device may be busy)")
                return False
            else:
                print(f"Unexpected response: {response_str}")
                return False
        except Exception as e:
            print(f"Failed to prepare update: {e}")
            return False
    
    async def start_update(self) -> bool:
        """Start OTA update process"""
        if not self.connected:
            print("Not connected to device")
            return False
        
        try:
            print("Starting OTA update...")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"START_UPDATE")
            await asyncio.sleep(0.5)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_STARTED" in response_str:
                print("OTA update started, ready to receive firmware")
                return True
            else:
                print(f"ERROR: Failed to start update: {response_str}")
                return False
        except Exception as e:
            print(f"Failed to start update: {e}")
            return False
    
    async def send_firmware_size(self, size: int) -> bool:
        """Send firmware size metadata"""
        if not self.connected:
            return False
        
        try:
            size_msg = f"SIZE:{size}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, size_msg)
            await asyncio.sleep(0.2)
            print(f"Sent firmware size: {size} bytes")
            return True
        except Exception as e:
            print(f"Failed to send firmware size: {e}")
            return False
    
    async def send_firmware_chunks(self, firmware_data: bytes, md5_hash: str) -> bool:
        """Send firmware in chunks"""
        if not self.connected:
            return False
        
        total_size = len(firmware_data)
        chunks_sent = 0
        total_chunks = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
        
        print(f"Sending firmware in {total_chunks} chunks...")
        
        try:
            # Send firmware chunks
            for i in range(0, total_size, CHUNK_SIZE):
                chunk = firmware_data[i:i + CHUNK_SIZE]
                await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, chunk)
                chunks_sent += 1
                
                # Calculate and display progress
                progress = (chunks_sent * 100) // total_chunks
                if progress != self.update_progress:
                    self.update_progress = progress
                    print(f"Progress: {progress}% ({chunks_sent}/{total_chunks} chunks)")
                
                # Small delay to prevent overwhelming BLE
                await asyncio.sleep(0.05)
            
            # Send MD5 hash after all chunks
            print("Sending MD5 hash...")
            md5_msg = f"MD5:{md5_hash}".encode('utf-8')
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, md5_msg)
            await asyncio.sleep(0.5)
            
            print("All firmware chunks sent successfully")
            return True
        except Exception as e:
            print(f"Failed to send firmware chunks: {e}")
            return False
    
    async def finalize_update(self) -> bool:
        """Finalize OTA update"""
        if not self.connected:
            return False
        
        try:
            print("Finalizing update...")
            await self.client.write_gatt_char(UPDATE_CHARACTERISTIC_UUID, b"FINALIZE_UPDATE")
            await asyncio.sleep(1.0)
            
            # Read response
            response = await self.client.read_gatt_char(UPDATE_CHARACTERISTIC_UUID)
            response_str = response.decode('utf-8')
            
            if "UPDATE_COMPLETE" in response_str:
                print("Update finalized successfully! Device will reboot...")
                return True
            elif "UPDATE_VALIDATION_FAILED" in response_str:
                print("ERROR: Firmware validation failed (MD5 mismatch)")
                return False
            elif "UPDATE_ERROR" in response_str:
                print(f"ERROR: Update failed: {response_str}")
                return False
            else:
                print(f"Unexpected response: {response_str}")
                return False
        except Exception as e:
            print(f"Failed to finalize update: {e}")
            return False
    
    def calculate_md5(self, data: bytes) -> str:
        """Calculate MD5 hash of firmware"""
        md5_hash = hashlib.md5(data).hexdigest()
        return md5_hash
    
    async def update_firmware(self, firmware_path: str) -> bool:
        """Complete OTA update process"""
        try:
            # Read firmware file
            print(f"Reading firmware file: {firmware_path}")
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
            
            firmware_size = len(firmware_data)
            print(f"Firmware size: {firmware_size} bytes")
            
            # Check firmware size against OTA partition size (0x1C0000 = 1,835,008 bytes)
            MAX_OTA_PARTITION_SIZE = 0x1C0000  # 1.75 MB per partition
            if firmware_size > MAX_OTA_PARTITION_SIZE:
                print(f"WARNING: Firmware size ({firmware_size} bytes) exceeds OTA partition size ({MAX_OTA_PARTITION_SIZE} bytes)")
                print("Update may fail if firmware is too large!")
            else:
                print(f"Firmware size OK: {firmware_size / 1024 / 1024:.2f} MB (max: {MAX_OTA_PARTITION_SIZE / 1024 / 1024:.2f} MB)")
            
            # Calculate MD5 hash
            print("Calculating MD5 hash...")
            md5_hash = self.calculate_md5(firmware_data)
            print(f"MD5 hash: {md5_hash}")
            # Request device to enter OTA mode (write to main characteristic)
            print("Requesting device to enter OTA mode via main characteristic...")
            try:
                await self.client.write_gatt_char(CHARACTERISTIC_UUID, b"ENABLE_OTA")
                await asyncio.sleep(1.0)
            except Exception as e:
                print(f"Warning: could not request OTA enable: {e}")

            # After requesting OTA mode, reconnect to refresh services if necessary
            try:
                await self.disconnect()
                await asyncio.sleep(0.5)
                if not await self.connect(self.address):
                    print("ERROR: Failed to reconnect after requesting OTA mode")
                    return False
            except Exception:
                pass

            # Check version
            await self.check_version()
            
            # Prepare update
            if not await self.prepare_update():
                return False
            
            # Start update
            if not await self.start_update():
                return False
            
            # Send firmware size
            if not await self.send_firmware_size(firmware_size):
                return False
            
            # Send firmware chunks
            if not await self.send_firmware_chunks(firmware_data, md5_hash):
                return False
            
            # Finalize update
            if not await self.finalize_update():
                return False
            
            print("\n" + "="*60)
            print("OTA UPDATE COMPLETED SUCCESSFULLY!")
            print("="*60)
            print("The device will reboot with the new firmware.")
            print("You may need to reconnect after the device reboots.")
            return True
            
        except FileNotFoundError:
            print(f"ERROR: Firmware file not found: {firmware_path}")
            return False
        except Exception as e:
            print(f"ERROR: Update failed: {e}")
            return False


async def main():
    parser = argparse.ArgumentParser(description='OTA Update for ESP32 Toilet System')
    parser.add_argument('firmware', type=str, help='Path to firmware binary file (.bin)')
    parser.add_argument('--address', type=str, help='BLE device address (optional, will scan if not provided)')
    parser.add_argument('--check-version', action='store_true', help='Only check firmware version, do not update')
    
    args = parser.parse_args()
    
    updater = OTAUpdater()
    
    print("="*60)
    print("ESP32 Toilet System OTA Update")
    print("="*60)
    
    # Scan for device or use provided address
    if args.address:
        address = args.address
        print(f"Using provided address: {address}")
    else:
        address = await updater.scan_for_device()
        if not address:
            print("ERROR: Could not find device")
            return 1
    
    # Connect to device
    if not await updater.connect(address):
        print("ERROR: Failed to connect to device")
        return 1
    
    try:
        if args.check_version:
            # Only check version
            await updater.check_version()
        else:
            # Perform full update
            success = await updater.update_firmware(args.firmware)
            if not success:
                print("\n" + "="*60)
                print("OTA UPDATE FAILED!")
                print("="*60)
                print("The device should automatically rollback to the previous firmware.")
                return 1
    finally:
        await updater.disconnect()
    
    return 0


if __name__ == "__main__":
    # Check if bleak is installed
    try:
        import bleak
    except ImportError:
        print("ERROR: bleak library not installed.")
        print("Install it with: pip install bleak")
        sys.exit(1)
    
    # Run the update
    exit_code = asyncio.run(main())
    sys.exit(exit_code)


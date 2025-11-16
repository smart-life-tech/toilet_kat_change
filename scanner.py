from bleak import BleakScanner
import asyncio

a = asyncio.get_event_loop().run_until_complete(BleakScanner.discover(timeout=15.0))
for d in a:
    print(d)
import asyncio
from bleak import BleakClient
import threading

ESP32_ADDRESS = "48:e7:29:9f:b2:a2"
CONTROL_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BLEClient:
    def __init__(self, address):
        self.address = address
        self.client = None
        self.connected = False

    async def connect(self):
        self.client = BleakClient(self.address)
        try:
            await self.client.connect()
            self.connected = self.client.is_connected
            if self.connected:
                print("Connected to ESP32")
            else:
                print("Failed to connect to ESP32")
        except Exception as e:
            print(f"Error connecting to ESP32: {e}")

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected")

    def get_connection_status(self):
        return self.connected

def run_ble_client(address):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    ble_client = BLEClient(address)
    loop.run_until_complete(ble_client.connect())

    # Keep the connection alive
    try:
        while True:
            loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(ble_client.disconnect())
    finally:
        loop.close()

def start_ble_client_thread():
    thread = threading.Thread(target=run_ble_client, args=(ESP32_ADDRESS,))
    thread.start()
    return thread

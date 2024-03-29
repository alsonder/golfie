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
                await self.print_services()
            else:
                print("Failed to connect to ESP32")
        except Exception as e:
            print(f"Error connecting to ESP32: {e}")
            self.connected = False  # Ensure the connected flag is correctly updated on failure

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected")
        self.connected = False

    def get_connection_status(self):
        return self.connected
    
    async def print_services(self):
        services = await self.client.get_services()
        print("Services and Characteristics:")
        for service in services:
            print(f"Service: {service.uuid}")
            for characteristic in service.characteristics:
                print(f"\tCharacteristic: {characteristic.uuid} ({', '.join(characteristic.properties)})")

async def run_ble_client(ble_client):
    # Ensure initial connection attempt outside of the loop to establish the connection as soon as possible
    await ble_client.connect()
    while True:
        try:
            if not ble_client.get_connection_status():
                print("Attempting to reconnect...")
                await ble_client.connect()
            await asyncio.sleep(10)  # Check connection status every 10 seconds
        except Exception as e:
            print(f"Error during reconnection: {e}")

def start_ble_client_thread(ble_client):
    def thread_target():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(run_ble_client(ble_client))

    thread = threading.Thread(target=thread_target)
    thread.start()
    return thread
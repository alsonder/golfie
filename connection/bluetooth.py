import asyncio
from bleak import BleakClient
import threading

ESP32_ADDRESS = "30:C9:22:12:58:72"  # ESP's unique MAC address
CONTROL_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # Hardcoded UUID

class BLEClient:
    def __init__(self, address):
        self.address = address
        self.client = None
        self.connected = False
        self.connect_event = asyncio.Event()

    async def connect(self):
        self.client = BleakClient(self.address)
        try:
            await self.client.connect()
            self.connected = self.client.is_connected
            if self.connected:
                print("Connected to ESP32")
                await self.print_services()  # Print characteristic values of the BLE server
                self.connect_event.set()
            else:
                print("Failed to connect to ESP32")
        except Exception as e:
            print(f"Error connecting to ESP32: {e}")
            self.connected = False  # Ensure the connected flag is correctly updated on failure
            self.connect_event.clear()

    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected")
        self.connected = False
        self.connect_event.clear()

    def get_connection_status(self):  # Function to check connection status
        return self.connected

    async def print_services(self):
        services = await self.client.get_services()
        print("Services and Characteristics:")
        for service in services:
            print(f"Service: {service.uuid}")
            for characteristic in service.characteristics:
                print(f"\tCharacteristic: {characteristic.uuid} ({', '.join(characteristic.properties)})")

    async def wait_for_connection(self, timeout=30):
        """Waits for the device to connect within the given timeout period."""
        start_time = asyncio.get_event_loop().time()
        while not self.connected:
            if (asyncio.get_event_loop().time() - start_time) > timeout:
                raise TimeoutError("Connection to BLE device timed out.")
            await asyncio.sleep(0.1)

async def run_ble_client(ble_client):
    # Ensure initial connection attempt outside of the loop to establish the connection as soon as possible
    await ble_client.connect()
    while True:  # Reconnect loop, blocking operation
        try:
            if not ble_client.get_connection_status():  # BLE reconnect logic
                print("Attempting to reconnect...")
                await ble_client.connect()
            await asyncio.sleep(10)  # Check connection status every 10 seconds
        except Exception as e:
            print(f"Error during reconnection: {e}")

def start_ble_client_thread(ble_client):
    def thread_target():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ble_client.loop = loop  # Store the event loop in the BLEClient instance for later access
        loop.run_until_complete(run_ble_client(ble_client))

    thread = threading.Thread(target=thread_target)
    thread.start()
    return thread  # Return the thread object

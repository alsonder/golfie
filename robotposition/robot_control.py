import asyncio
import keyboard
from bleak import BleakScanner, BleakClient

ESP32_NAME = "ESP_ballsucking_contraption"
CONTROL_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
control_keys = ['w', 'a', 's', 'd', 'f', 'g', 'r', 'c']  # wasd selfexplanatory.. g=blow, f= suck, r=stop fans, c = stop motors
current_key = None
key_pressed = False
stop_command_sent = False  # Flag to ensure stop command is sent only once per key release

def on_key_event(event):
    global current_key, key_pressed, stop_command_sent
    if event.name in control_keys:
        if event.event_type == "down" and not key_pressed:
            current_key = event.name
            key_pressed = True
            stop_command_sent = False  # Reset on new key press
        elif event.event_type == "up":
            key_pressed = False

keyboard.hook(on_key_event)

async def send_stop_command(client):
    global stop_command_sent
    if not stop_command_sent:
        stop_command = 'c'  # stop command hard coded in the ESP program
        await client.write_gatt_char(CONTROL_CHARACTERISTIC_UUID, stop_command.encode())
        print("Sent stop command.")
        stop_command_sent = True

async def find_device(name):
    devices = await BleakScanner.discover(timeout=20)
    for device in devices:
        if device.name == name:
            return device
    return None

async def send_command_loop(device_address):
    global current_key, key_pressed
    client = BleakClient(device_address)
    try:
        await client.connect()
        print(f"Connected to {device_address}")
        while True:
            if key_pressed and current_key:
                await client.write_gatt_char(CONTROL_CHARACTERISTIC_UUID, current_key.encode())
                print(f"Sent command: {current_key}")
            elif not key_pressed:
                await send_stop_command(client)
            await asyncio.sleep(0.1)
    except Exception as e:
        print(e)
    finally:
        if client.is_connected:
            await client.disconnect()
            print("Disconnected.")

async def main():
    print("Scanning for ESP32 device...")
    device = await find_device(ESP32_NAME)
    if device:
        print(f"Found ESP32 device with address {device.address}")
        await send_command_loop(device.address)
    else:
        print(f"ESP32 device named '{ESP32_NAME}' not found.")

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(main())
except KeyboardInterrupt:
    print("Program exited by user.")
finally:
    loop.close()
    keyboard.unhook_all()  

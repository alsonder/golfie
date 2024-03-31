import asyncio

CONTROL_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8" # Hardcoded UUID


class MotorControl:
    def __init__(self, ble_client):
        self.ble_client = ble_client  # Use the existing BLEClient instance
    
    async def send_command(self, command):
        if not self.ble_client.connected:
            print("BLE Client is not connected. Command not sent.")
            return
        try:
            await self.ble_client.client.write_gatt_char(CONTROL_CHARACTERISTIC_UUID, command.encode())
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Error sending command {command}: {e}")

    async def move_forward(self):
        await self.send_command('w')

    async def move_backward(self):
        await self.send_command('s')

    async def turn_left(self):
        await self.send_command('a')

    async def turn_right(self):
        await self.send_command('d')

    async def stop_movement(self):
        await self.send_command('c')

    async def suction_on(self):
        await self.send_command('f')

    async def blow_on(self):
        await self.send_command('g')

    async def fans_off(self):
        await self.send_command('r')

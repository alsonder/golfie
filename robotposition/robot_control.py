import asyncio
import threading
import queue

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
        
    async def set_pwm_motor_a(self, pwm_value):
        """Set PWM for motor A."""
        command = f"p {pwm_value}"
        await self.send_command(command)

    async def set_pwm_motor_b(self, pwm_value):
        """Set PWM for motor B."""
        command = f"q {pwm_value}"
        await self.send_command(command)

class RobotMovement:
    def __init__(self, motor_control):
        self.motor_control = motor_control  # Use the MotorControl instance passed during initialization
        self.command_queue = queue.Queue()
        self.loop = asyncio.new_event_loop()
        
    def start_event_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self.process_commands())
        self.loop.run_forever()

    async def process_commands(self):
        while True:
            command, args = await self.loop.run_in_executor(None, self.command_queue.get)
            try:
                # Ensure command execution within the RobotMovement's event loop
                await command(*args)
            except Exception as e:
                print(f"Error executing command: {e}")

    def start(self):
        threading.Thread(target=self.start_event_loop, daemon=True).start()

    def stop(self):
        self.loop.call_soon_threadsafe(self.loop.stop)

    def send_command(self, command, *args):
        # Command is a method from MotorControl, args are the arguments for that method
        self.command_queue.put((command, args))

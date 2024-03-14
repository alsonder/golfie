import bluetooth

target_name = "ESP32"   # target device name
target_address = None

nearby_devices = bluetooth.discover_devices()

# scanning for target device
for bdaddr in nearby_devices:
    if target_name == bluetooth.lookup_name(bdaddr):
        target_address = bdaddr
        break

if target_address is not None:
    print(f"found target bluetooth device with address {target_address}")
else:
    print("could not find target bluetooth device nearby")

# establishing the connection
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((target_address, 1))  # the second parameter is the port, adjust if needed

# communication example
sock.send("Hello ESP32!")
data = sock.recv(1024)  # receive up to 1KB of data
print(f"Received: {data}")

sock.close()
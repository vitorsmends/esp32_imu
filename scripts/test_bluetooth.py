import serial
import time

# Replace 'COMx' with your actual Bluetooth COM port (e.g., 'COM5' on Windows, '/dev/ttyUSB0' on Linux)
bluetooth_port = '/dev/rfcomm0'
baud_rate = 115200  # The baud rate should match the one set on the ESP32

# Initialize the serial connection
ser = serial.Serial(bluetooth_port, baud_rate)

print("Connected to ESP32")

try:
    while True:
        if ser.in_waiting > 0:
            # Read the incoming data from the ESP32
            data = ser.readline().decode('utf-8').rstrip()
            print(f"Received: {data}")
        time.sleep(1)  # Optional delay to make the output more readable

except KeyboardInterrupt:
    print("Closing connection")
    ser.close()


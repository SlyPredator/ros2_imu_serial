import serial

# Change these to match your setup
# PORT = "COM3"        # Windows example: COM3
PORT = "/dev/ttyUSB0"  # Linux
# PORT = "/dev/tty.usbserial-XXXX"  # macOS
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Listening for serial data... (Ctrl+C to stop)")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline()
            print("Received:", data)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    ser.close()

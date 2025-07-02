import serial
import struct

serial = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.1)

while True:
    raw = serial.read(8)
    print("RX:", ' '.join(f'{byte:02X}' for byte in raw))

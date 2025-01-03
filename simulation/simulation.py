import serial
import time
import random
import serial.tools.list_ports
import struct
import threading

buffer = ""

# Initialize Serial
def find_usb_device():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usbs' in port.device or 'COM' in port.device:
            return port.device
    raise Exception("No USB serial device found")


ser = serial.Serial(find_usb_device(), 230400, timeout=1)
print(f"Connected to {ser.name}")
time.sleep(2)  # Allow board to reset

while True:
    # Read all available data in the buffer
    if ser.in_waiting > 0:
        buffer += ser.read(ser.in_waiting).decode('utf-8')
        
        # Process lines if newline received
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            
            if line == 'REQ_DATA':
                ser.write(b"0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,10,0.5,5.2,0.53\n")
                #print("Data request received")
            elif line == "PING":
                ser.write(b"TRUE\n")
            elif line:
                print(f"IN: {line}")
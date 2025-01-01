import serial
import time
import random

# Initialize Serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Allow board to reset

# Enter Simulation Mode
ser.write(b"SIM\n")

# Simulated Sensor Data Generation
def simulate_sensors():
    x = random.randint(-200, 200)  # Simulate acceleration
    y = random.randint(-200, 200)
    z = random.randint(900, 1100)  # Simulate 1g
    pressure = random.randint(100000, 102000)  # Simulated pressure (Pa)
    return x, y, z, pressure

while True:
    # Listen for STM32 requests
    request = ser.readline().decode().strip()
    
    if request == "REQ_ACCEL":
        x, y, z, _ = simulate_sensors()
        ser.write(f"{x},{y},{z}\n".encode())

    elif request == "REQ_PRESSURE":
        _, _, _, pressure = simulate_sensors()
        ser.write(f"{pressure}\n".encode())

    time.sleep(0.05)  # 50 Hz update rate

import serial
import time
import random
import serial.tools.list_ports
import struct
import threading
import numpy as np

buffer = ""

def injest_open_rocket_sim(filename):
    #TODO: open rocket file

    # TODO: Best data structure to hold this?


        # Load CSV into DataFrame
    array = np.genfromtxt(f"OpenRocket Sims/{filename}.csv", delimiter=",", dtype=None, encoding=None)

    sensor_data = [np.empty((len(array), 12), dtype=float)]

    for row in array:
        time = row[0]
        altitude = row[1]
        vertical_vel = row[2]
        vertical_accel = row[3]
        air_temperature = row[4]
        air_pressure = row[5]

        x = 0
        y = vertical_accel / 9.81
        z = 0
        mode = 10
        position = 0
        velocity = 0
        current = 0

        data_frame = np.array([time, x, x, y, y, z, z, air_temperature, air_pressure, mode, position, velocity, current])
        sensor_data = np.vstack([sensor_data, data_frame])

    #Add random noise distribution to data
    noise_345 = np.random.normal(0, 0.003, len(sensor_data))
    noise_375 = np.random.normal(0, 0.03, len(sensor_data))
    noise_pressure = np.random.normal(0, 0.04, len(sensor_data))
    noise_temperature = np.random.normal(0, 0.03, len(sensor_data))

    sensor_data[:, 1] += noise_345
    sensor_data[:, 2] += noise_375
    sensor_data[:, 3] += noise_345
    sensor_data[:, 4] += noise_375
    sensor_data[:, 5] += noise_345
    sensor_data[:, 6] += noise_375
    sensor_data[:, 7] += noise_temperature
    sensor_data[:, 8] += noise_pressure

    np.savetxt('Sim Outputs/filename.csv', array, delimiter=',', fmt='%d')


injest_open_rocket_sim("Test_2")

# # Initialize Serial
# def find_usb_device():
#     ports = serial.tools.list_ports.comports()
#     for port in ports:
#         if 'usbs' in port.device or 'COM' in port.device:
#             return port.device
#     raise Exception("No USB serial device found")


# ser = serial.Serial(find_usb_device(), 230400, timeout=1)
# print(f"Connected to {ser.name}")
# time.sleep(2)  # Allow board to reset

# while True:
#     # Read all available data in the buffer
#     if ser.in_waiting > 0:
#         buffer += ser.read(ser.in_waiting).decode('utf-8')
        
#         # Process lines if newline received
#         while '\n' in buffer:
#             line, buffer = buffer.split('\n', 1)
#             line = line.strip()
            
#             if line == 'REQ_DATA':
#                 ser.write(b"0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,10,0.5,5.2,0.53\n")
#                 #print("Data request received")
#             elif line == "PING":
#                 ser.write(b"TRUE\n")
#             elif line:
#                 print(f"IN: {line}")
                
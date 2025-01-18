import serial
import time
import random
import serial.tools.list_ports
import struct
import threading
import numpy as np

buffer = ""

# def injest_open_rocket_sim(filename):
#     #TODO: open rocket file

#     # TODO: Best data structure to hold this?


#         # Load CSV into DataFrame
#     array = np.genfromtxt(f"OpenRocket Sims/{filename}.csv", delimiter=",", dtype=float, encoding=None)

#     sensor_data = np.empty((0, 6), dtype=float)

#     for i in range(0, 99): #10 seconds of idle data
#         time = i / 10
#         x = 0
#         y = 1
#         z = 0
#         air_temperature = 15
#         air_pressure = 1012.963

#         data_frame = np.array([time, x, y, z, air_temperature, air_pressure])
#         data_frame = data_frame.reshape(1, -1)  # Make it a 2D array with one row
#         sensor_data = np.vstack([sensor_data, data_frame])

#     for row in array:
#         time = row[0] + 10
#         altitude = row[1]
#         vertical_vel = row[2]
#         vertical_accel = row[3]
#         air_temperature = row[4]
#         air_pressure = row[5] / 100

#         x = 0
#         y = (vertical_accel / 9.81) + 1
#         z = 0
#         mode = 10
#         position = 0
#         velocity = 0
#         current = 0

#         data_frame = np.array([time, x, y, z, air_temperature, air_pressure])
#         data_frame = data_frame.reshape(1, -1)  # Make it a 2D array with one row
#         sensor_data = np.vstack([sensor_data, data_frame])

#     #Add random noise distribution to data
#     noise_345 = np.random.normal(0, 0.003, len(sensor_data))
#     noise_375 = np.random.normal(0, 0.03, len(sensor_data))
#     noise_pressure = np.random.normal(0, 0.04, len(sensor_data))
#     noise_temperature = np.random.normal(0, 0.03, len(sensor_data))

#     sensor_data[:, 1] += noise_345
#     sensor_data[:, 2] += noise_345
#     sensor_data[:, 3] += noise_345
#     sensor_data[:, 4] += noise_temperature
#     sensor_data[:, 5] += noise_pressure

#     np.savetxt(f'Sim Outputs/{filename}.csv', sensor_data, delimiter=',', fmt='%.3f')


# injest_open_rocket_sim("Test_2")

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

array = np.genfromtxt(f"Sims Output/Test_2.csv", delimiter=",", dtype=float, encoding=None)
time_column = array[:, 0]  # Extract the time column

start_time = time.time()

while True:

    # Read all available data in the buffer
    if ser.in_waiting > 0:
        buffer += ser.read(ser.in_waiting).decode('utf-8')
        
        # Process lines if newline received
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            
            if line == 'REQ_DATA':

                elapsed_time = time.time() - start_time

                # Find the closest timestep in the array
                closest_index = (np.abs(time_column - elapsed_time)).argmin()
                closest_row = array[closest_index]  # Extract the closest row

                dataframe = np.array([closest_row[0], closest_row[1], closest_row[2], closest_row[3], closest_row[4], closest_row[5]])

                #Convert dataframe to string
                data_string = ','.join(map(str, dataframe))

                ser.write(data_string + "\n")
                #print("Data request received")
            elif line == "PING":
                ser.write(b"TRUE\n")
            elif line:
                print(f"IN: {line}")
                
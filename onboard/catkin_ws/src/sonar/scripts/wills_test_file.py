from brping import Ping360
import numpy as np
# from sonar import Sonar
from sonar_image_processing import build_sonar_image_from_int_array
import matplotlib.pyplot as plt
import cv2

# BAUD_RATE = 2000000  # hz
# SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
# SPEED_OF_SOUND_IN_WATER = 1482  # m/s

# # number of values to filter TODO figure out where the noise ends
# FILTER_INDEX = 100 # first 100 values are filtered out
# DEFAULT_RANGE = 5 # m
# DEFAULT_NUMER_OF_SAMPLES = 1200 # 1200 is max resolution

# SONAR_FTDI_OOGWAY = "DK0C1WF7"


# def range_to_period(range):
#     return round(2 * range / (number_of_samples * SPEED_OF_SOUND_IN_WATER * SAMPLE_PERIOD_TICK_DURATION))

# def range_to_transmit(range):
#     transmit_duration = 8000 * range / SPEED_OF_SOUND_IN_WATER
#     transmit_duration = round(max(range_to_period(range) / 40, transmit_duration))
#     return max(5, min(500, transmit_duration))

# def meters_per_sample():
#     # sample_period is in 25ns increments
#     # time of flight includes there and back, so divide by 2
#     return SPEED_OF_SOUND_IN_WATER * sample_period * SAMPLE_PERIOD_TICK_DURATION / 2.0

# def get_distance_of_sample(sample_index):
#     # 0.5 for the average distance of sample
#     return (sample_index + 0.5) * meters_per_sample()

# _serial_port = None

# while _serial_port is None:
#     try:
#         _serial_port = next(list_ports.grep(SONAR_FTDI_OOGWAY)).device
#     except StopIteration:
#         print('error')

# print(_serial_port)

# ping360 = Ping360()

# ping360.connect_serial(f'{_serial_port}', BAUD_RATE)
# ping360.initialize()

# number_of_samples = DEFAULT_NUMER_OF_SAMPLES
# ping360.set_number_of_samples(number_of_samples)

# sample_period = range_to_period(DEFAULT_RANGE)
# ping360.set_sample_period(sample_period)

# transmit_duration = range_to_transmit(DEFAULT_RANGE)
# ping360.set_transmit_duration(transmit_duration)
# print(f"period: {sample_period}, transmit duration: {transmit_duration}")

# response = ping360.transmitAngle(200)
# response_to_int_array = [int(item) for item in response.data]
# filtered_sonar_sweep = [int(0)] * FILTER_INDEX + response_to_int_array[FILTER_INDEX:]

# def request_data_at_angle(angle_in_gradians):
#     response = ping360.transmitAngle(angle_in_gradians)
#     response_to_int_array = [int(item) for item in response.data] # converts bytestring to int array
#     filtered_sonar_scan = [int(0)] * FILTER_INDEX + response_to_int_array[FILTER_INDEX:] # replaces first FILTER_INDEX values with 0
#     return filtered_sonar_scan

# sonar_sweep_data = []
# for i in range(195, 201):
#     sonar_scan = request_data_at_angle(i)
#     sonar_sweep_data.append(sonar_scan)

# np_array = np.vstack(sonar_sweep_data)
# np.savetxt("my_array.txt", np_array, fmt='%d', delimiter=', ')

# column_sums = np.sum(np_array, axis=0)
# np.savetxt("my_array2.txt", column_sums, fmt='%d', delimiter=', ')
# x_average_pos = np.sum(np.multiply(column_sums, np.arange(0, column_sums.size)))/np.sum(column_sums)

# row_sums = np.sum(np_array, axis=1)
# np.savetxt("my_array3.txt", row_sums, fmt='%d', delimiter=', ')
# y_average_pos = np.sum(np.multiply(row_sums, np.arange(0, row_sums.size)))/np.sum(row_sums)

# print(f"{x_average_pos} {y_average_pos}")

# sonar = Sonar()

# data = sonar.get_sweep(100,300)
# data = build_sonar_image_from_int_array(data)

#import numpy array from onboard\catkin_ws\src\sonar\scripts\sampleData\gater5.npy
data = np.load('onboard\catkin_ws\src\sonar\scripts\sampleData\gater5.npy')
print(data)

data = cv2.cvtColor(data.astype(np.uint8), cv2.COLOR_GRAY2BGR)
data = cv2.applyColorMap(data, cv2.COLORMAP_VIRIDIS)
cv2.imshow("image", data)
cv2.waitKey(0)
# pyright: reportMissingImports=false, reportUnusedImport=false, reportUnboundVariable=false, reportUndefinedVariable=false
# flake8: noqa
"""
This script performs underwater color correction on images from a DepthAI camera.
It must be run on the DepthAI device itself, using a script node in the pipeline.
See depthai_spatial_detection.py for an example.
"""

"""
Reverse engineering notes:

Helpful links:
https://discuss.luxonis.com/d/825-get-depth-value-in-script-embeded-in-camera

node.io['cam_rgb'].get() is an lpb.ImgFrame object with the following attributes:
['Type', '__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__',
'__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__',
'__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__',
'__sizeof__', '__str__', '__subclasshook__', 'getCategory', 'getColorTemperature', 'getData',
'getExposureTime', 'getHeight', 'getInstanceNum', 'getLensPosition', 'getLensPositionRaw',
'getSensitivity', 'getSequenceNum', 'getTimestamp', 'getTimestampDevice', 'getType', 'getWidth',
'setCategory', 'setData', 'setHeight', 'setInstanceNum', 'setSequenceNum', 'setSize', 'setTimestamp',
'setType', 'setWidth']

len(getData()) is 519168
This is the size of the image, which is 416 * 416 * 3 = 519168 (height * width * channels)

getData() is a memoryview object (see https://docs.python.org/3/library/stdtypes.html#memoryview)
"""

import math

THRESHOLD_RATIO = 2000
MIN_AVG_RED = 60
MAX_HUE_SHIFT = 120
BLUE_MAGIC_VALUE = 1.2
SAMPLE_SECONDS = 2  # Extracts color correction from every N seconds

channel_size = 416 * 416

def print(val):
    node.warn(str(val))

# Function to calculate the average of a list of integers
def calculate_average(channel_data):
    return sum(channel_data) // len(channel_data)

# Assuming `data` is your bytearray and `channel_size` is the size of each channel
def get_channel_averages(data):
    # Extract each channel's data
    channel_1 = data[:channel_size]
    channel_2 = data[channel_size:channel_size*2]
    channel_3 = data[channel_size*2:channel_size*3]

    # Calculate the average for each channel
    avg_channel_1 = calculate_average(channel_1)
    avg_channel_2 = calculate_average(channel_2)
    avg_channel_3 = calculate_average(channel_3)

    # Return the averages as a tuple
    return (avg_channel_1, avg_channel_2, avg_channel_3)

def hue_shift_red(data, h):
    U = math.cos(h * math.pi / 180)
    W = math.sin(h * math.pi / 180)

    c1 = 0.299 + 0.701 * U + 0.168 * W
    c2 = 0.587 - 0.587 * U + 0.330 * W
    c3 = 0.114 - 0.114 * U - 0.497 * W

    shifted_data = bytearray(len(data))

    for i in range(channel_size):
        r = data[i]
        g = data[i + channel_size]
        b = data[i + 2 * channel_size]

        shifted_data[i] = min(255, max(0, int(c1 * r)))
        shifted_data[i + channel_size] = min(255, max(0, int(c2 * g)))
        shifted_data[i + 2 * channel_size] = min(255, max(0, int(c3 * b)))

    return shifted_data

def get_correction(data: memoryview):
    avg_mat = get_channel_averages(data)

    new_avg_r = avg_mat[2]  # Red channel is at index 2
    hue_shift = 0

    while new_avg_r < MIN_AVG_RED:
        shifted = hue_shift_red(data, hue_shift)
        new_avg_r = calculate_average(shifted[len(data)//3*2:])  # Calculate average of the red channel in shifted data
        hue_shift += 1
        if hue_shift > MAX_HUE_SHIFT:
            new_avg_r = MIN_AVG_RED

    print(hue_shift)
    print(new_avg_r)

def set_color(data: memoryview, channel: 0 | 1 | 2):
    # blue = 0
    # green = 1
    # red = 2

    # Calculate start and end indices for the current channel
    start_index = channel_size * channel
    end_index = start_index + channel_size

    # Create a new bytearray with the required values
    new_data = bytearray(len(data))

    # Use slicing to set the values more efficiently
    new_data[:start_index] = b'\x00' * start_index
    new_data[start_index:end_index] = b'\xff' * channel_size
    new_data[end_index:] = b'\x00' * (len(data) - end_index)

    # Update the original data
    return new_data


while True:
    frame: 'lpb.ImgFrame' = node.io['cam_rgb'].get()
    data: memoryview = frame.getData()

    # new_data = set_color(data, channel=0)
    new_data = get_correction(data)
    # frame.setData(new_data)

    node.io['spatial_detection_network'].send(frame)

    print('Processed frame')

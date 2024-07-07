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

channel_size = 416 * 416

def print(val):
    node.warn(str(val))

def set_color(data: memoryview, channel: 0 | 1 | 2):
    # blue = 0
    # green = 1
    # red = 2
    for i in range(0, len(data)):
        if i >= channel_size * channel and i < channel_size * (channel+1):
            data[i] = 255
        else:
            data[i] = 0

while True:
    frame: 'lpb.ImgFrame' = node.io['cam_rgb'].get()
    data: memoryview = frame.getData()

    set_color(data, channel=0)

    node.io['spatial_detection_network'].send(frame)

    print('Processed frame')

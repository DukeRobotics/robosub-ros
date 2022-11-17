#!/usr/bin/env python3

import math
import struct, sys, re
import numpy as np
from PIL import ImageFilter
import matplotlib.pyplot as plt
from scipy import signal
import cv2
# 3.7 for dataclasses, 3.8 for walrus (:=) in recovery
assert (sys.version_info.major >= 3 and sys.version_info.minor >= 8), \
    "Python version should be at least 3.8."

from brping import PingParser, PingMessage
from dataclasses import dataclass
from typing import IO, Any, Set
from localSonar import Sonar

def indent(obj, by=' '*4):
    return by + str(obj).replace('\n', f'\n{by}')


@dataclass
class PingViewerBuildInfo:
    hash_commit: str = ''
    date: str = ''
    tag: str = ''
    os_name: str = ''
    os_version: str = ''

    def __str__(self):
        return f"""PingViewerBuildInfo:
    hash: {self.hash_commit}
    date: {self.date}
    tag: {self.tag}
    os:
        name: {self.os_name}
        version: {self.os_version}
    """


@dataclass
class Sensor:
    family: int = 0
    type_sensor: int = 0

    def __str__(self):
        return f"""Sensor:
    Family: {self.family}
    Type: {self.type_sensor}
    """


@dataclass
class Header:
    string: str = ''
    version: int = 0
    ping_viewer_build_info = PingViewerBuildInfo()
    sensor = Sensor()

    def __str__(self):
        return f"""Header:
    String: {self.string}
    Version: {self.version}
    {indent(self.ping_viewer_build_info)}
    {indent(self.sensor)}
    """


class PingViewerLogReader:
    ''' Structured as a big-endian sequence of
        size: uint32, data: byte_array[size].
    '''

    # int32 values used in message header
    INT = struct.Struct('>i')
    # big-endian uint32 'size' parsed for every timestamp and message
    #  -> only compile and calcsize once
    UINT = struct.Struct('>I')
    # NOTE: ping-viewer message buffer length is 10240 (from pingparserext.h)
    #  should we use this instead??
    # longest possible ping message (id:2310) w/ 1200 samples
    #  -> double the length in case windows used UTF-16
    MAX_ARRAY_LENGTH = 1220*2
    # timestamp format for recovery hh:mm:ss.xxx
    # includes optional \x00 (null byte) before every character because Windows
    TIMESTAMP_FORMAT = re.compile(
        b'(\x00?\d){2}(\x00?:\x00?[0-5]\x00?\d){2}\x00?\.(\x00?\d){3}')
    MAX_TIMESTAMP_LENGTH = 12 * 2

    def __init__(self, filename: str):
        self.filename = filename
        self.header = Header()
        self.messages = []

    @classmethod
    def unpack_int(cls, file: IO[Any]):
        data = file.read(cls.INT.size)
        return cls.INT.unpack_from(data)[0]

    @classmethod
    def unpack_uint(cls, file: IO[Any]):
        ''' String and data array lengths. '''
        data = file.read(cls.UINT.size)
        return cls.UINT.unpack_from(data)[0]

    @classmethod
    def unpack_array(cls, file: IO[Any]):
        ''' Returns the unpacked array if <= MAX_ARRAY_LENGTH, else None. '''
        array_size = cls.unpack_uint(file)
        if array_size <= cls.MAX_ARRAY_LENGTH:
            return file.read(array_size)

    @classmethod
    def unpack_string(cls, file: IO[Any]):
        return cls.unpack_array(file).decode('UTF-8')

    @classmethod
    def unpack_message(cls, file: IO[Any]):
        timestamp = cls.unpack_string(file)
        message = cls.unpack_array(file)
        if message is None:
            return cls.recover(file)
        return (timestamp, message)

    @classmethod
    def recover(cls, file: IO[Any]):
        """ Attempt to recover from a failed read.

        Assumed that a bad number has been read from the last cls.UINT.size
        set of bytes -> try to recover by seeking 'file' back to there, then
        read until the next timestamp, and continue as normal from there.

        """
        file.seek(current_pos := (file.tell() - cls.UINT.size))
        prev_ = next_ = b''
        start = amount_read = 0
        while not (match := cls.TIMESTAMP_FORMAT.search(
                roi := (prev_ + next_), start)):
            prev_ = next_
            next_ = file.read(cls.MAX_ARRAY_LENGTH)
            if not next_:
                break # run out of file
            amount_read += cls.MAX_ARRAY_LENGTH
            if start == 0 and prev_:
                # onto the second read
                # -> match on potential overlap + new region, not the
                #    already-checked (impossible) region
                start = cls.MAX_ARRAY_LENGTH - cls.MAX_TIMESTAMP_LENGTH
        else:
            # match was found
            end = match.end()
            timestamp = roi[match.start():end].decode('UTF-8')
            # return the file pointer to the end of this timestamp
            file.seek(current_pos + amount_read - (len(roi) - end))
            # attempt to extract the corresponding message, or recover anew
            if (message := cls.unpack_array(file)) is None:
                return cls.recover(file)
            return (timestamp, message)
        raise EOFError('No timestamp match found in recovery attempt')

    def unpack_header(self, file: IO[Any]):
        self.header.string = self.unpack_string(file)
        self.header.version = self.unpack_int(file)

        for info in ('hash_commit', 'date', 'tag', 'os_name', 'os_version'):
            setattr(self.header.ping_viewer_build_info, info,
                    self.unpack_string(file))

        self.header.sensor.family = self.unpack_int(file)
        self.header.sensor.type_sensor = self.unpack_int(file)

    def process(self):
        """ Process and store the entire file into self.messages. """
        self.messages.extend(self)

    def __iter__(self):
        """ Creates an iterator for efficient reading of self.filename.

        Yields (timestamp, message) pairs for decoding.

        """
        with open(self.filename, "rb") as file:
            self.unpack_header(file)
            while "data available":
                try:
                    yield self.unpack_message(file)
                except struct.error:
                    break # reading complete

    def parser(self, message_ids: Set[int] = {1300, 2300, 2301}):
        """ Returns a generator that parses and decodes this log's messages.

        Yields (timestamp, message) pairs. message decoded as a PingMessage.

        'message_ids' is the set of Ping Profile message ids to filter by.
            Default value is {1300, 2300, 2301} -> {Ping1D.profile,
                                                    Ping360.device_data,
                                                    Ping360.auto_device_data}

        """
        self._parser = PingParser()

        for (timestamp, message) in self:
            # parse each byte of the message
            for byte in message:
                # Check if the parser has registered and verified this message
                if self._parser.parse_byte(byte) == self._parser.NEW_MESSAGE:
                    # Get decoded message
                    decoded_message = self._parser.rx_msg
                    if decoded_message.message_id in message_ids:
                        yield timestamp, decoded_message
                        break # this message is (should be?) over, get next one
                # else message is still being parsed


@dataclass(init=False, order=True)
class Ping1DSettings:
    transmit_duration : int # [us]
    scan_start        : int # [mm] from transducer
    scan_length       : int # [mm]
    gain_setting      : int # [0-6 -> 0.6-144]

    def __init__(self, profile: PingMessage):
        for attr in self.__annotations__:
            setattr(self, attr, getattr(profile, attr))

    @property
    def gain(self):
        """ Returns device receiver 'gain', as specified by 
        https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/#1300-profile.
        
        """
        assert 0 <= self.gain_setting <= 6, "Invalid gain value."
        return (0.6, 1.8, 5.5, 12.9, 30.2, 66.1, 144)[self.gain_setting]


@dataclass(init=False, order=True)
class Ping360Settings:
    mode               : int # Ping360 = 1
    gain_setting       : int # 0-2 (low,normal,high)
    transmit_duration  : int # 1-1000 [us]
    sample_period      : int # 80-40_000 [25ns]
    transmit_frequency : int # [kHz]
    number_of_samples  : int # per ping

    def __init__(self, device_data: PingMessage):
        for attr in self.__annotations__:
            setattr(self, attr, getattr(device_data, attr))

    @property
    def gain(self):
        assert 0 <= self.gain_setting <= 2, "Invalid gain value."
        return ('low', 'medium', 'high')[self.gain_setting]

    @property
    def sample_period_us(self):
        """ Returns device sample period in microseconds. """
        assert 80 <= self.sample_period <= 40_000, "Invalid sample period."
        return self.sample_period * 25e-3
    
    def meters_per_sample(self, v_sound):
        """ Returns the distance [m] covered by each sample of a ping.

        Distance depends on 'v_sound' [m/s], the speed of sound in the
            surrounding liquid.

        """
        # time of flight -> v_sound * (there + back) / 2
        return v_sound * self.sample_period_us * 1e-6 / 2


def increase_brightness(img, value=20):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def find_gate_posts(img): 
    
    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_image = cv2.imread("C:\\Users\\willd\\DukeRobotics\\robosub-ros\\onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Image2.jpeg", cv2.IMREAD_COLOR)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image,5) # blur image

    lower_color_bounds = (40,80,0) # filter out lower values (ie blue)
    upper_color_bounds = (230,250,255) #filter out too high values
    mask = cv2.inRange(cm_image,lower_color_bounds,upper_color_bounds)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cm_circles = sorted(cm_circles, key=cv2.contourArea, reverse=True)
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 200), cm_circles)) 
    cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cm_circles)) 

    #return if not both goal posts found 
    if(len(cm_circles) < 1):
        print("not enough circles found")
        return None

    filtered_circles = cm_circles[0:2]

    circle_positions = []
    for circle in filtered_circles:  #find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX,cY))
        #print("object at " + "x: " + str(cX) + "  Y: " + str(cY)  +  " has circularity : " + str(perimeter**2/ (4*math.pi*area) ) )
        #cv2.circle(cm_copy_image, (cX, cY), 3, (255, 255, 255), -1)

    cv2.drawContours(cm_copy_image, filtered_circles, -1, (0,255,0), 2)
    cv2.imshow("image", cm_copy_image)
    cv2.waitKey(0)

    return circle_positions

def find_buouy(img): 
    
    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_image = cv2.imread("C:\\Users\\willd\\DukeRobotics\\robosub-ros\\onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Image3.jpeg", cv2.IMREAD_COLOR)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image,5) # blur image

    lower_color_bounds = (40,80,0) # filter out lower values (ie blue)
    upper_color_bounds = (230,250,255) #filter out too high values
    mask = cv2.inRange(cm_image,lower_color_bounds,upper_color_bounds)

    cv2.imshow("image", mask)
    cv2.waitKey(0)


def getdecodedfile(localfilename):
    import os
    dirname = os.path.dirname(__file__)
    
    filename =  dirname + localfilename

    # Open log and begin processing
    log = PingViewerLogReader("\\sampleData\\"+filename)

    return log.parser()

if __name__ == "__main__":

    import os
    dirname = os.path.dirname(__file__)
    
    filename =  dirname + '\\sampleData\\SampleTylerData.bin'

    # Open log and begin processing
    log = PingViewerLogReader(filename)

    for index, (timestamp, decoded_message) in enumerate(log.parser()):
        
        if(index >= 49 and index <= 149):
            split_bytes = [decoded_message.data[i:i+1] for i in range(len(decoded_message.data))]
            split_bytes = split_bytes[100:]
            byte_from_int = int.from_bytes(split_bytes[0], "big")
            intarray = np.array([byte_from_int])
            for i in range(len(split_bytes) -1):
                byte_from_int = int.from_bytes(split_bytes[i+1], "big")
                #if(byte_from_int <= 90):
                    #byte_from_int = 0
                intarray = np.append(intarray, [byte_from_int])
            if(index == 49):
                sonar_matrix = np.asarray(intarray)
            else:
                sonar_matrix = np.vstack((sonar_matrix, intarray))

    print("finding posts")
    found_posts = find_gate_posts(sonar_matrix)
    print(found_posts)
    plt.imsave('onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Image.jpeg', sonar_matrix)

        
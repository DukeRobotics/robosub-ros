#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from spike_detection import PipelineConfig, spikes_pipeline, get_first_hydrophones
from record import SaleaeCapture
import os
import time

CENTER_FREQUENCY = 35000
FREQ_RADIUS = 1000

config = PipelineConfig(CENTER_FREQUENCY - FREQ_RADIUS, CENTER_FREQUENCY + FREQ_RADIUS)
config.noise_floor_stdev_mult = 5.5

saleae = SaleaeCapture()

def localization_pub():

    SAMPLE_DURATION = 2 # seconds
    DIRECTORY = '/home/duke-robotics/robosub-ros/onboard/catkin_ws/src/acoustics/data'

    # -1 means we are under the octagon
    # 0-3 inclusive are the 4 quadrants relative to the robot heading

    pub = rospy.Publisher('/acoustics/localization', Int8, queue_size=10)
    rospy.init_node('localization_pub', anonymous=True)

    while not rospy.is_shutdown():
        
        saleae.capture(SAMPLE_DURATION, DIRECTORY)

        search_for = os.path.join(DIRECTORY, 'analog_0.bin')

        while not os.path.exists(search_for):
            time.sleep(0.25)

        spikes = spikes_pipeline(DIRECTORY, config)
        if len(spikes) == 0:
            rospy.loginfo('No spikes detected')
            continue
        firsts = get_first_hydrophones(spikes, config)
        
        times = [s.time for s in spikes[-1]]
        range = max(times) - min(times)
        if range < 1e-5:
            counts = -1
        else:
            counts = [0, 0, 0, 0]
            for i in firsts:
                counts[i] += 1
            localization = counts.index(max(counts))
            
        
        
        rospy.loginfo(localization)
        pub.publish(localization)

if __name__ == '__main__':
    try:
        localization_pub()
    except rospy.ROSInterruptException:
        pass


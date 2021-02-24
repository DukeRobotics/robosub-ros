import pandas
import math
import numpy as np
import rospy
import os
import sys


def dis(x1, y1, z1, x, y, z):
    return math.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)

class DataGenerator:
    VS = 1511.5  # velocity of sound

    def __init__(self, sampling_freq, pinger_freq, hydrophone, pinger_loc, publish):
        self.sf = sampling_freq
        self.pinger_frequency = pinger_freq
        self.publish = publish
        self.samples = int(math.floor(2.048 * 2 * self.sf))
        self.pinger_loc = pinger_loc
        self.hydrophone = hydrophone
        if hydrophone == 'cheap':
            space = .3
            self.hp1 = [0, 0, 0]
            self.hp2 = [space, 0, 0]
            self.hp3 = [0, space, 0]
            self.hp4 = [0, 0, space]
        elif hydrophone == 'expensive':
            space = 0.0115
            self.hp1 = [0, 0, 0]
            self.hp2 = [0, -space, 0]
            self.hp3 = [-space, 0, 0]
            self.hp4 = [-space, -space, 0]
        else:
            rospy.logerr('Invalid hydrophone type, set hydrophone to "cheap" or "expensive"')

    def run(self):
        dis1 = dis(self.hp1[0], self.hp1[1], self.hp1[2], *self.pinger_loc)
        dis2 = dis(self.hp2[0], self.hp2[1], self.hp2[2], *self.pinger_loc)
        dis3 = dis(self.hp3[0], self.hp3[1], self.hp3[2], *self.pinger_loc)
        dis4 = dis(self.hp4[0], self.hp4[1], self.hp4[2], *self.pinger_loc)

        # hydrophone channel data
        ping1 = np.zeros(self.samples)
        ping2 = np.zeros(self.samples)
        ping3 = np.zeros(self.samples)
        ping4 = np.zeros(self.samples)

        # ping
        s = np.arange(0, math.floor(0.004 * self.sf))
        ping = 0.1 * np.sin((s / self.sf * self.pinger_frequency * 2 * math.pi))  # complex conjugate transpose (np.matrix.H)

        # samples until ping occurs
        buffer = 40000

        ping1[int(math.ceil(dis1 / self.VS * self.sf)) + buffer: int(math.ceil((dis1 / self.VS + 0.004) * self.sf)) + buffer] = ping

        ping2[int(math.ceil(dis2 / self.VS * self.sf)) + buffer: int(math.ceil((dis2 / self.VS + 0.004) * self.sf)) + buffer] = ping

        ping3[int(math.ceil(dis3 / self.VS * self.sf)) + buffer: int(math.ceil((dis3 / self.VS + 0.004) * self.sf)) + buffer] = ping

        ping4[int(math.ceil(dis4 / self.VS * self.sf)) + buffer: int(math.ceil((dis4 / self.VS + 0.004) * self.sf)) + buffer] = ping

        ping1[int(math.ceil(dis1 / self.VS * self.sf + 2.048 * self.sf)) + buffer:
              int(math.ceil((dis1 / self.VS + 0.004) * self.sf + 2.048 * self.sf)) + buffer] = ping

        ping2[int(math.ceil(dis2 / self.VS * self.sf + 2.048 * self.sf)) + buffer:
              int(math.ceil((dis2 / self.VS + 0.004) * self.sf + 2.048 * self.sf)) + buffer] = ping

        ping3[int(math.ceil(dis3 / self.VS * self.sf + 2.048 * self.sf)) + buffer:
              int(math.ceil((dis3 / self.VS + 0.004) * self.sf + 2.048 * self.sf)) + buffer] = ping

        ping4[int(math.ceil(dis4 / self.VS * self.sf + 2.048 * self.sf)) + buffer:
              int(math.ceil((dis4 / self.VS + 0.004) * self.sf + 2.048 * self.sf)) + buffer] = ping

        for i in range(1, 5):
            mean = 0
            std = 0.01

            h1 = ping1 + np.random.normal(mean, std, self.samples)
            h2 = ping2 + np.random.normal(mean, std, self.samples)
            h3 = ping3 + np.random.normal(mean, std, self.samples)
            h4 = ping4 + np.random.normal(mean, std, self.samples)

            df = pandas.DataFrame({'Channel 0': h1, 'Channel 1': h2, 'Channel 2': h3, 'Channel 3': h4})
            filepath = os.path.join(sys.path[0], '../data', '../data/simulated-%s_%d_%d_%d_(%d).csv' % (self.hydrophone, self.pinger_loc[0], self.pinger_loc[1], self.pinger_loc[2], i))
            df.to_csv(filepath)
            self.publish(curr_file=i, total_file=4)

        return True

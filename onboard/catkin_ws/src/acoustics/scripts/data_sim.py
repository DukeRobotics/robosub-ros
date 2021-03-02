import pandas
import math
import numpy as np
import numpy.linalg
import resource_retriever as rr
import os
import sys

class DataGenerator:
    VS = 1511.5  # velocity of sound

    def __init__(self, sampling_freq, pinger_freq, hydrophone, pinger_loc, publish):
        self.sf = sampling_freq
        self.pinger_frequency = pinger_freq
        self.publish = publish
        self.samples = int(math.floor(2.048 * 2 * self.sf))
        self.pinger_loc = pinger_loc
        self.hydrophone = hydrophone
        if self.hydrophone == 'cheap':
            space = .3
            self.hp = [[0, 0, 0],
                       [space, 0, 0],
                       [0, space, 0],
                       [0, 0, space]]
        elif self.hydrophone == 'expensive':
            space = 0.0115
            self.hp = [[0, 0, 0],
                       [0, -space, 0],
                       [-space, 0, 0],
                       [-space, -space, 0]]
        else:
            self.hp = None
            self.publish(curr_file=0, total_file=4, msg="Failure: incorrect hydrophone type on data generation")

    def run(self):
        if self.hp is None:
            return []

        dis = [np.linalg.norm(np.array(h) - np.array(self.pinger_loc)) for h in self.hp]

        # hydrophone channel data
        pings = [np.zeros(self.samples) for i in range(4)]

        # ping
        s = np.arange(0, math.floor(0.004 * self.sf))
        ping = 0.1 * np.sin((s / self.sf * self.pinger_frequency * 2 * math.pi))  # complex conjugate transpose (np.matrix.H)

        # samples until ping occurs
        buffer = 40000

        for i in range(4):
            start_i = int(math.ceil(dis[i] / self.VS * self.sf)) + buffer
            end_i = int(math.ceil((dis[i] / self.VS + 0.004) * self.sf)) + buffer
            pings[i][start_i: end_i] = ping

            start_i = int(math.ceil(dis[i] / self.VS * self.sf + 2.048 * self.sf)) + buffer
            end_i =  int(math.ceil((dis[i] / self.VS + 0.004) * self.sf + 2.048 * self.sf)) + buffer
            pings[i][start_i: end_i] = ping

        file_paths = []
        for i in range(1, 5):
            mean = 0
            std = 0.01
            hs = [p + np.random.normal(mean, std, self.samples) for p in pings]

            df = pandas.DataFrame({'Channel 0': hs[0], 'Channel 1': hs[1], 'Channel 2': hs[2], 'Channel 3': hs[3]})
            filepath = "package://acoustics/data/simulated_{}_{}_{}_{}_({}).csv".format(self.hydrophone, self.pinger_loc[0], self.pinger_loc[1], self.pinger_loc[2], i)
            df.to_csv(rr.get_filename(filepath, use_protocol=False))
            file_paths.append(filepath)
            self.publish(curr_file=i, total_file=4, msg="File generation successful, moving onto next file")

        return file_paths

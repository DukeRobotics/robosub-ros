import numpy as np
import pandas as pd
from scipy.signal import butter, lfilter, freqz, correlate
import sys
import os
from acoustics_math import read_data, butter_bandpass_filter, plot_filter


class AcousticGuess:
    BAND_WIDTH = 500

    def __init__(self, filepath, fs, freq, publish):
        self.filepath = filepath
        self.fs = fs
        self.freq = freq
        self.publish = publish

    def run(self):
        self.publish(curr_stage=1, total_stage=3)

        data = read_data(self.filepath)
        lowcut = self.freq - self.BAND_WIDTH//2
        highcut = self.freq + self.BAND_WIDTH//2
        #plot_filter(lowcut, highcut, fs, 4)

        filtered = [butter_bandpass_filter(channel, lowcut, highcut, self.fs, order=4) for channel in data]
        filtered = [np.absolute(channel).tolist() for channel in filtered]

        self.publish(curr_stage=2, total_stage=3)

        cross_corr = [correlate(filtered[0], channel, mode='full') for channel in filtered[1:]]

        self.publish(curr_stage=3, total_stage=3)

        coord = [np.argmax(cross) - len(filtered[0]) for cross in cross_corr]
        #print(coord)

        octant = [1 if i > 0 else -1 for i in coord]
        return octant

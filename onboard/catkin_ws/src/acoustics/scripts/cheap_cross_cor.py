import numpy as np
import pandas as pd
#import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, freqz, correlate
import sys
import os

'''
cheap hydrophone location
space = .3;
hp1 = [0,0,0];
hp2 = [space, 0,0];
hp3 = [0, space, 0];
hp4 = [0, 0, space];
'''

#filepath = '../Data/matlab_custom_cheap_hydrophone_(1).csv'
# filepath = '../Data/matlab_custom_cheap_hydrophone_-2_7_4_(1).csv'
# fs = 625000
# freq = 40000


class AcousticGuess:
    ping_duration = .004  # how long the ping lasts in seconds
    band_width = 500

    def __init__(self, filepath, fs, freq, publish):
        self.filepath = os.path.join(sys.path[0], '../data', filepath)
        self.fs = fs
        self.freq = freq
        self.publish = publish

        self.octant = [0, 0, 0]

    def run(self):
        data = read_data(self.filepath)

        lowcut = self.freq-self.band_width//2
        highcut = self.freq+self.band_width//2
        #plot_filter(lowcut, highcut, fs, 4)

        f1 = butter_bandpass_filter(data[0], lowcut, highcut, self.fs, order=4)
        f2 = butter_bandpass_filter(data[1], lowcut, highcut, self.fs, order=4)
        f3 = butter_bandpass_filter(data[2], lowcut, highcut, self.fs, order=4)
        f4 = butter_bandpass_filter(data[3], lowcut, highcut, self.fs, order=4)
        #print(len(f1))
        #plot([c1, f1])

        self.publish(filtered=True, cross_correlated=False)

        f1 = np.absolute(f1).tolist()
        f2 = np.absolute(f2).tolist()
        f3 = np.absolute(f3).tolist()
        f4 = np.absolute(f4).tolist()

        cross12 = correlate(f1, f2, mode='full')  # ex: 2560060 - 2560000 = 60. c1 is about 60 samples later than c2
        cross13 = correlate(f1, f3, mode='full')
        cross14 = correlate(f1, f4, mode='full')
        #print(len(cross12))

        self.publish(filtered=True, cross_correlated=True)

        x = np.argmax(cross12) - len(f1)
        y = np.argmax(cross13) - len(f1)
        z = np.argmax(cross14) - len(f1)
        #print(x, y, z)

        self.octant[0] = 1 if x > 0 else -1
        self.octant[1] = 1 if y > 0 else -1
        self.octant[2] = 1 if z > 0 else -1
        #print(self.octant)

        return self.octant

def read_data(filepath):
    df = pd.read_csv(filepath, skiprows=[1], skipinitialspace=True)
    return [df["Channel 0"].tolist(), df["Channel 1"].tolist(), df["Channel 2"].tolist(), df["Channel 3"].tolist()]

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y


# obj = AcousticGuess(filepath, fs, freq)
# print(obj.run())

'''
def max_moving_avg(data, window_size):
    window = np.ones(window_size)/window_size
    avgs = np.convolve(data, window, mode='same')
    return np.argmax(avgs)

def plot_filter(lowcut, highcut, fs, order):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    w, h = freqz(b, a, worN=2000)
    plt.plot((fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % order)
    plt.plot([0, 0.5 * fs], [np.sqrt(0.5), np.sqrt(0.5)], '--', label='sqrt(0.5)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain')
    plt.grid(True)
    plt.legend(loc='best')
    plt.show()

def plot(data):
    if len(data) == 1:
        plt.plot(data[0])
    else:
        fig, axs = plt.subplots(len(data))
        for i, ax in enumerate(axs.flat):
            ax.plot(data[i])
    plt.show()
'''

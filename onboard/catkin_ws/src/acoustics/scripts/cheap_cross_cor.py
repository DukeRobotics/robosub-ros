import numpy as np
from scipy.signal import correlate
from acoustics_math import read_data, butter_bandpass_filter
import resource_retriever as rr


class AcousticGuess:
    BAND_WIDTH = 500

    def __init__(self, file_paths, fs, freq, publish):
        self.file_paths = [rr.get_filename(fpath, use_protocol=False) for fpath in file_paths]
        self.fs = fs
        self.freq = freq
        self.publish = publish

    def run(self):
        octants = {}
        for i in range(len(self.file_paths)):
            data = read_data(self.file_paths[i])
            lowcut = self.freq - self.BAND_WIDTH//2
            highcut = self.freq + self.BAND_WIDTH//2
            # plot_filter(lowcut, highcut, fs, 4)

            filtered = [butter_bandpass_filter(channel, lowcut, highcut, self.fs, order=4) for channel in data]
            filtered = [np.absolute(channel).tolist() for channel in filtered]

            cross_corr = [correlate(filtered[0], channel, mode='full') for channel in filtered[1:]]

            coord = [np.argmax(cross) - len(filtered[0]) for cross in cross_corr]

            octant = tuple(1 if i > 0 else -1 for i in coord)
            if octant not in octants:
                octants[octant] = 0
            octants[octant] += 1

            self.publish(curr_stage=i+1, total_stages=len(self.file_paths), msg="Finished file, moving onto next one")

        return max(octants, key=lambda k: octants[k])

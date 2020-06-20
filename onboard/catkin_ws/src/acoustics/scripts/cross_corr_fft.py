# to adjust this script to different noise environment,
# change fft_w_size, large_window_portion, and invalidation requirement in get_pdiff
import numpy as np
import acoustics_math as ac
import sys
import os


class AcousticProcessor:

    VELOCITY_SOUND = 1511.5
    FFT_W_SIZE = 125
    CHECK_LEN = 20
    LARGE_WINDOW_PORTION = 5
    SPAC = 0.0115

    def __init__(self, filename, if_double, version, fs, freq, guess, publish_counts, if_plot=False):
        self.filename = os.path.join(sys.path[0], '../data', filename)
        self.if_double = if_double
        self.version = version
        self.fs = fs
        self.pingc = self.fs * 0.004
        self.freq = freq
        self.guess = guess
        self.publish_counts = publish_counts
        self.if_plot = if_plot
        self.hp = [np.array([0, 0, 0]),
                   np.array([0, -self.SPAC, 0]),
                   np.array([-self.SPAC, 0, 0]),
                   np.array([-self.SPAC, -self.SPAC, 0])]
        self.ccwha = []
        self.downva = []
        self.actual = []
        self.process_count = 0
        self.success_count = 0

    def process_data(self, raw_data):
        data = np.array([ac.split_data(d) if self.if_double else [d] for d in raw_data])

        for j in range(len(data[0])):
            pdiff = ac.data_to_pdiff(data[:, j],
                                     self.freq, self.pingc, self.FFT_W_SIZE, self.LARGE_WINDOW_PORTION, self.fs)
            self.process_count += 1

            if None not in pdiff:
                self.success_count += 1
                diff = [(val / 2 / np.pi * self.VELOCITY_SOUND / self.freq) for val in pdiff]

                ans = ac.solver(self.guess, diff, self.hp)
                x, y, z = ans[0], ans[1], ans[2]
                self.actual.append((x, y, z))

                # calculate angle
                ccwh = np.arctan2(y, x)
                self.ccwha.append(ccwh)
                downv = np.arctan2(-z, np.sqrt(x ** 2 + y ** 2))
                self.downva.append(downv)

                # compare solver result with pdiff from data
                # pd = ac.check_angle(self.hp, ccwh, downv, self.VELOCITY_SOUND, self.freq, self.CHECK_LEN)
                # print("checked pd", pd[0], pd[1], pd[2])
                # print("pdiff_12", pdiff[0], "pdiff_13", pdiff[1], "pdiff_34", pdiff[2], "\n")
            else:
                print("invalid acoustics data\n")
            self.publish_counts(self.success_count, self.process_count)

    def run(self):

        if self.version == 0:
            filenames = [self.filename]
        else:
            filenames = [self.filename.replace(".csv", "("+str(i+1)+").csv") for i in range(self.version)]

        for data_file in filenames:
            self.process_data(ac.read_data(data_file))

        final_ccwh, valid_count = ac.final_hz_angle(self.ccwha)

        if self.if_plot:
            ac.plot(self.ccwha, self.downva, valid_count, self.actual, self.guess)

        return final_ccwh, valid_count, self.success_count

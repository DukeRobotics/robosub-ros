import pandas
import numpy as np
from scipy.optimize import fsolve
from scipy.signal import butter, lfilter, freqz
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa
from mpl_toolkits.mplot3d import Axes3D  # noqa


def get_pdiff(parr1, parr2, start, end, large_window_portion):
    pdllist = np.subtract(parr2, parr1)
    pdlist = correct_phase(pdllist[start:end])
    var = variance_list(pdlist, int(len(pdlist) / large_window_portion))
    # var = variance_list(pdlist, int(len(pdlist)/3))
    phase_start = np.argmin(var)
    # check if lowest variance align with max mag interval, if not then bad data
    # print("phase start", phase_start+start)
    # print("start", start)
    # plt.figure()
    # plt.plot(correct_phase(pdllist))
    # plt.title("phase diff")
    # plt.show()
    # THIS REQUIREMENT CAN BE LOSEN IF TOO MANY PINGS ARE INVALID, YET MIGHT LEAD TO INACCURATE RESULT. Change 2 to 1.5.
    if phase_start > len(pdlist) / 2:
        return None
    phase_end = phase_start + int(len(pdlist) / large_window_portion)
    return np.mean(pdlist[phase_start:phase_end])


def reduce_phase(phase):
    return -phase / abs(phase) * (2 * np.pi - abs(phase))


def correct_phase(arr):
    return [reduce_phase(phase) if abs(phase) > np.pi else phase for phase in arr]


def moving_average_max(a, pingc, fft_w_size, n=None):
    if n is None:
        n = int(pingc / fft_w_size)
    weights = np.repeat(1.0, n) / n
    return np.argmax(np.convolve(a, weights, 'valid'))


def fft_sw(xn, freq, fs):
    n = np.arange(len(xn))
    exp = np.exp(-1j * freq / fs * 2 * np.pi * n)
    return np.dot(xn, exp)


def fft(xn, freq, w_size, fs):
    ft = []
    for i in range(int(len(xn) / w_size)):
        xn_s = xn[i * w_size:(i + 1) * w_size]
        ft.append(fft_sw(xn_s, freq, fs))
    return np.angle(ft), np.absolute(ft)


def variance_list(arr, window):
    return [np.var(arr[i:i + window]) for i in range(len(arr) - window + 1)]


def apply_to_pairs(fn, arr):
    return [fn(arr[0], arr[1]), fn(arr[0], arr[2]), fn(arr[2], arr[3])]


def diff_equation(hp_a, hp_b, target, t_diff):
    return (np.linalg.norm(target - hp_a) - np.linalg.norm(target - hp_b)) - t_diff


def system(target, *data):
    hp, diff_12, diff_13, diff_34 = data
    return (diff_equation(hp[0], hp[1], target, diff_12),
            diff_equation(hp[0], hp[2], target, diff_13),
            diff_equation(hp[2], hp[3], target, diff_34))


def solver(guess, diff, hp):
    data_val = (hp, diff[0], diff[1], diff[2])
    return fsolve(system, guess, args=data_val)


def data_to_pdiff(data, freq, pingc, fft_w_size, large_window_portion, fs):
    plist_all, mlist_all = zip(*[fft(d, freq, fft_w_size, fs) for d in data])
    mlist = np.sum(mlist_all, axis=0)
    mag_start = moving_average_max(mlist, pingc, fft_w_size)
    mag_end = mag_start + int(pingc / fft_w_size)
    return apply_to_pairs(lambda p1, p2: get_pdiff(p1, p2, mag_start, mag_end, large_window_portion), plist_all)


def read_data(filepath):
    df = pandas.read_csv(filepath, skiprows=[1], skipinitialspace=True)
    return [df["Channel 0"].tolist(), df["Channel 1"].tolist(), df["Channel 2"].tolist(), df["Channel 3"].tolist()]


def split_data(data):
    return [data[0:int(len(data) / 2)], data[int(len(data) / 2):len(data)]]


def check_angle(hp, hz, vt, vsound, freq, check_len):
    pinger = np.array([check_len * np.cos(vt) * np.cos(hz), check_len *
                      np.cos(vt) * np.sin(hz), -check_len * np.sin(vt)])
    dis = [np.linalg.norm(pinger - hp_val) for hp_val in hp]
    return apply_to_pairs(lambda dis1, dis2: (dis1 - dis2) / vsound * freq * 2 * np.pi, dis)


def plot(ccwha, downva, count, actual, guess):
    print("\nsuccess count: ", count, "\n")
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter3D([guess[0]], [guess[1]], [guess[2]], color="b")
    ax.scatter3D([val[0] for val in actual], [val[1] for val in actual], [val[2] for val in actual], color="r")
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    for i in range(len(ccwha)):
        plot_3d(ccwha[i], downva[i], ax)
    plt.show()


def plot_3d(ccwh, downv, ax):
    if abs(ccwh) < np.pi / 2:
        xline = np.linspace(0, 10, 100)
    else:
        xline = np.linspace(-10, 0, 100)
    yline = xline * np.tan(ccwh)
    zline = -xline / np.cos(ccwh) * np.tan(downv)
    ax.plot3D(xline, yline, zline, 'gray')


def final_hz_angle(hz_arr):
    quadrant = [[], [], [], []]
    quadrant[0] = [hz for hz in hz_arr if 0 <= hz <= np.pi / 2]
    quadrant[1] = [hz for hz in hz_arr if hz > np.pi / 2]
    quadrant[2] = [hz for hz in hz_arr if hz <= -np.pi / 2]
    quadrant[3] = [hz for hz in hz_arr if 0 > hz > -np.pi / 2]
    max_len = np.max([len(each) for each in quadrant])
    ave = np.array([q for q in quadrant if len(q) == max_len]).flatten()
    return np.mean(ave), len(ave)


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

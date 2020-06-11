# to adjust this script to different noise environment,
# change fft_w_size, large_window_portion, and invalidation requirement in get_pdiff

import pandas
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# sampling frequency
fs = 625000
# number of sample taken during the ping
pingc = fs*0.004
# target frequency
freq = 40000
# speed of sound in water
vsound = 1511.5
# nipple distance between hydrophone
spac = 0.0115
# allowed phase diff
dphase = np.pi*2/(vsound/freq)*spac

fft_w_size = 125

check_len = 20

large_window_portion = 5

guess = (0, 0, -10)

hp = [np.array([0, 0, 0]),
      np.array([0, -spac, 0]),
      np.array([-spac, 0, 0]),
      np.array([-spac, -spac, 0])]
# potential future configuration
# hp2 = np.array([-spac/2, -np.sqrt(3)*spac/2, 0])
# hp3 = np.array([-spac, 0, 0])
# hp4 = np.array([-spac/2, -np.sqrt(3)*spac/4, -3*spac/4])


def get_pdiff(parr1, parr2, start, end):
    pdllist = np.subtract(parr2, parr1)
    pdlist = correct_phase(pdllist[start:end])
    var = variance_list(pdlist, int(len(pdlist)//large_window_portion))
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
    if phase_start > len(pdlist)/2:
        return None
    phase_end = phase_start + int(len(pdlist)//large_window_portion)
    return np.mean(pdlist[phase_start:phase_end])

def reduce_phase(phase):
    return -phase/abs(phase)*(2*np.pi-abs(phase))

def correct_phase(arr):
    return [reduce_phase(phase) if abs(phase) > np.pi else phase for phase in arr]

def fft_sw(xn, freq):
    n = np.arange(len(xn))
    exp = np.exp(-1j*freq/fs*2*np.pi*n)
    return np.dot(xn, exp)

def get_alist(a, n):
    weights = np.repeat(1.0, n)/n
    return np.convolve(a, weights, 'valid')

def moving_average_max(a, n = int(pingc/fft_w_size)):
    return np.argmax(get_alist(a, n))

def fft(xn, freq, w_size):
    ft = []
    for i in range(int(len(xn)//w_size)):
        xn_s = xn[i*w_size:(i+1)*w_size]
        ft.append(fft_sw(xn_s, freq))
    return np.angle(ft), np.absolute(ft)

def variance_list(arr, window):
    return [np.var(arr[i:i+window]) for i in range(len(arr) - window + 1)]

def apply_to_pairs(fn, arr):
    return [fn(arr[0], arr[1]), fn(arr[0], arr[2]), fn(arr[2], arr[3])]

def diff_equation(hp_a, hp_b, target, t_diff):
    return (np.linalg.norm(target-hp_a) - np.linalg.norm(target-hp_b)) - t_diff

def system(target, *data):
    diff_12, diff_13, diff_34 = data
    return (diff_equation(hp[0], hp[1], target, diff_12),
            diff_equation(hp[0], hp[2], target, diff_13),
            diff_equation(hp[2], hp[3], target, diff_34))

def solver(guess, diff):
    data_val = (diff[0], diff[1], diff[2])
    return fsolve(system, guess, args=data_val)

def data_to_pdiff(data):
    plist_all, mlist_all = zip(*[fft(d, freq, fft_w_size) for d in data])
    mlist = np.sum(mlist_all, axis=0)
    mag_start = moving_average_max(mlist)
    mag_end = mag_start + int(pingc//fft_w_size)
    return apply_to_pairs(lambda p1, p2: get_pdiff(p1, p2, mag_start, mag_end), plist_all)

def read_data(filepath):
    df = pandas.read_csv(filepath, skiprows=[1], skipinitialspace=True)
    print("running ", filepath)
    return [df["Channel 0"].tolist(), df["Channel 1"].tolist(), df["Channel 2"].tolist(), df["Channel 3"].tolist()]

def split_data(data):
    return [data[0:int(len(data)/2)], data[int(len(data)/2):len(data)]]

def check_angle(hz, vt):
    pinger = np.array([check_len*np.cos(vt)*np.cos(hz), check_len*np.cos(vt)*np.sin(hz), -check_len*np.sin(vt)])
    dis = [np.linalg.norm(pinger-hp_val) for hp_val in hp]
    return apply_to_pairs(lambda dis1, dis2: (dis1 - dis2)/vsound*freq*2*np.pi, dis)

def plot_3d(ccwh, downv, ax):
    if abs(ccwh) < np.pi/2:
        xline = np.linspace(0, 10, 100)
    else:
        xline = np.linspace(-10, 0, 100)
    yline = xline*np.tan(ccwh)
    zline = -xline/np.cos(ccwh)*np.tan(downv)
    ax.plot3D(xline, yline, zline, 'gray')

def plot(ccwha, downva, count, actual):
    print("\nsuccess count: ", count, "\n")
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter3D([guess[0]], [guess[1]], [guess[2]], color="b")
    # ax.scatter3D([val[0] for val in actual], [val[1] for val in actual], [val[2] for val in actual], color="r")
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    for i in range(len(ccwha)):
        plot_3d(ccwha[i], downva[i], ax)
    plt.show()

def final_hz_angle(hz_arr):
    quadrant = [[], [], [], []]
    quadrant[0] = [hz for hz in hz_arr if 0 <= hz <= np.pi/2]
    quadrant[1] = [hz for hz in hz_arr if hz > np.pi/2]
    quadrant[2] = [hz for hz in hz_arr if hz <= -np.pi/2]
    quadrant[3] = [hz for hz in hz_arr if 0 > hz > -np.pi/2]
    max_len = np.max([len(each) for each in quadrant])
    ave = np.array([q for q in quadrant if len(q) == max_len]).flatten()
    return np.mean(ave), len(ave)

def process_data(raw_data, if_double, actual, ccwha, downva, count):
    data = np.array([split_data(d) if if_double else [d] for d in raw_data])

    for j in range(len(data[0])):
        pdiff = data_to_pdiff(data[:, j])

        if None not in pdiff:
            count += 1
            diff = [(val / 2 / np.pi * vsound / freq) for val in pdiff]

            ans = solver(guess, diff)
            x, y, z = ans[0], ans[1], ans[2]
            actual.append((x, y, z))
            print("initial guess", guess)
            print("x, y, z", actual[-1])

            # calculate angle
            ccwh = np.arctan2(y, x)
            ccwha.append(ccwh)
            print("horizontal angle", np.rad2deg(ccwh))
            downv = np.arctan2(-z, np.sqrt(x ** 2 + y ** 2))
            downva.append(downv)
            print("vertical downward angle", np.rad2deg(downv), "\n")

            # compare solver result with pdiff from data
            pd = check_angle(ccwh, downv)
            print("checked pd", pd[0], pd[1], pd[2])
            print("pdiff_12", pdiff[0], "pdiff_13", pdiff[1], "pdiff_34", pdiff[2], "\n")
        else:
            print("invalid\n")
    return actual, ccwha, downva, count


def cross_corr_func(filename, if_double, version, if_plot, samp_f=fs, tar_f=freq, guess_x=guess[0], guess_y=guess[1], guess_z=guess[2]):
    global fs, freq, guess
    fs = samp_f
    freq = tar_f
    filepath = filename
    guess = (guess_x, guess_y, guess_z)

    ccwha = []
    downva = []
    actual = []
    count = 0

    if version == 0:
        raw_data = read_data(filepath)
        actual, ccwha, downva, count = process_data(raw_data, if_double, actual, ccwha, downva, count)

    for i in range(version):
        raw_data = read_data(filepath.replace(".csv", "("+str(i+1)+").csv"))
        actual, ccwha, downva, count = process_data(raw_data, if_double, actual, ccwha, downva, count)

    final_ccwh, valid_count = final_hz_angle(ccwha)
    print("\n\nfinal horizontal angle", np.rad2deg(final_ccwh))
    print("valid count", valid_count)

    if if_plot:
        plot(ccwha, downva, count, actual)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        cross_corr_func(sys.argv[1], True, 4, True, 625000, 40000, 0, 0, -10)
    else:
        try:
            cross_corr_func(sys.argv[1], sys.argv[2] == "True", int(sys.argv[3]), sys.argv[4] == "True", int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), int(sys.argv[8]), int(sys.argv[9]))
        except:
            print("wrong input arguments")

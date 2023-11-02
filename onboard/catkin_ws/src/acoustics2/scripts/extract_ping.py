import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from data_sim import gen_timeseries
import matplotlib as mpl

mpl.rcParams['agg.path.chunksize'] = 100000

LOWCUT = 30_000
HIGHCUT = 40_000

SAMPLE_RATE = 625_000


def butter_bandpass(lowcut, highcut, fs=SAMPLE_RATE, order=5):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut=LOWCUT, highcut=HIGHCUT, fs=SAMPLE_RATE, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Filter and detect peaks for each channel
def detect_wave_packet(channel_data):

    # Filter the channel data
    filtered_signal = butter_bandpass_filter(channel_data)

    # Find peaks in the filtered signal
    peaks, _ = find_peaks(filtered_signal, height=0.01, distance=100)

    return peaks, filtered_signal

def main():
    # Filter and detect peaks for all channels
    # timeseries = gen_timeseries(np.array([10,10,0]))

    # Read data from csv file
    data_0 = np.genfromtxt('sample1.csv', delimiter=',', skip_header=1)

    # Use only the first 20% of the data
    # Data is of shape n by 4

    peaks = []
    filtered_signals = []
    for i in range(3):
        channel_peaks, filtered_signal = detect_wave_packet(data_0[:,i+1])
        peaks.append(channel_peaks)
        filtered_signals.append(filtered_signal)

    print([len(filtered_signals[i]) for i in range(3)])


    
    # peaks = []
    # filtered_signals = []
    # for channel_data in timeseries:
    #     channel_peaks, filtered_signal = detect_wave_packet(channel_data)
    #     peaks.append(channel_peaks)
    #     filtered_signals.append(filtered_signal)

    print([len(peaks[i]) for i in range(3)])

    # Plot the detected peaks against both the raw and filtered signals
    # 2x4 subplots

    fig, ax = plt.subplots(3,2, figsize=(10,10))

    for i in range(3):
        # Plot the raw signal
        ax[i,0].plot(data_0[:,0], data_0[:,i+1])

        # Plot the filtered signal
        ax[i,1].plot(data_0[:,0], filtered_signals[i])

        # Plot the detected peaks
        ax[i,1].plot(data_0[peaks[i],0], filtered_signals[i][peaks[i]], "x")

        ax[i,0].set_title(f"Hydrophone {i}")
        ax[i,0].set_xlabel("Time (s)")
        ax[i,0].set_ylabel("Amplitude")
        ax[i,1].set_title(f"Hydrophone {i}")
        ax[i,1].set_xlabel("Time (s)")
        ax[i,1].set_ylabel("Amplitude")

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
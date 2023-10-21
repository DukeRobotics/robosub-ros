import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from data_sim import gen_timeseries, SAMPLE_RATE

LOWCUT = 30_000
HIGHCUT = 40_000

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
def detect_wave_packet(channel_data, channel_num):

    # Filter the channel data
    filtered_signal = butter_bandpass_filter(channel_data)

    # Find peaks in the filtered signal
    peaks, _ = find_peaks(filtered_signal, height=0.5, distance=100)

# Filter and detect peaks for all channels
timeseries = gen_timeseries(np.array([10,10,0]))

# Plot the raw data
plt.figure(figsize=(10, 6))
for channel_num, channel_data in enumerate(timeseries, 1):
    plt.subplot(2, 2, channel_num)
    plt.plot(channel_data)
    plt.title(f'Channel {channel_num} - Raw')

plt.tight_layout()
plt.show()

# Plot the filtered data
plt.figure(figsize=(10, 6))
for channel_num, channel_data in enumerate(timeseries, 1):
    plt.subplot(2, 2, channel_num)
    plt.plot(butter_bandpass_filter(channel_data))
    plt.title(f'Channel {channel_num} - Filtered')

plt.tight_layout()
plt.show()

import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from data_sim import gen_timeseries
import matplotlib as mpl
import time
from matplotlib import interactive

mpl.rcParams['agg.path.chunksize'] = 100_000

LOWCUT = 34_500
HIGHCUT = 35_500
PING_FREQ = 35_000
BANDPASS_WIDTHS = [4_000, 2_000, 1000, 750, 500]
BANDPASS_WIDTH = 1000
LOWPASS_CUTOFF = 6_000 # kinda arbitrarily chosen. In practice, cutoff for envelope detection should be square root of carrier freq (35 kHz) and message freq (not quite sure, chose 1 kHz?) TODO: Run fft on data and see what the frequency spectrum looks like

SAMPLE_RATE = 625_000

def butter_bandpass(lowcut, highcut, fs=SAMPLE_RATE, order=5):
    """
    Generates the coefficients for a Butterworth bandpass filter.

    Parameters
    ----------
    lowcut : int
        The lower frequency cutoff

    highcut : int
        The higher frequency cutoff

    fs : int
        The sample rate

    order : int
        The order of the filter

    Returns
    -------
    b : np.array
        The numerator coefficients

    a : np.array
        The denominator coefficients
    """

    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut=LOWCUT, highcut=HIGHCUT, fs=SAMPLE_RATE, order=5):
    """
    Filters the data using a Butterworth bandpass filter.
    
    Parameters
    ----------
    data : np.array
        The data to be filtered
        
    lowcut : int
        The lower frequency cutoff
    
    highcut : int
        The higher frequency cutoff
    
    fs : int
        The sample rate
    
    order : int
        The order of the filter
    """

    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_lowpass_filter(data, cutoff=LOWPASS_CUTOFF, fs = SAMPLE_RATE, order = 5):
    """
    Filters the data using a Butterworth bandpass filter.
    
    Parameters
    ----------
    data : np.array
        The data to be filtered
        
    cutoff : int
        The cutoff frequency of the filter
    
    fs : int
        The sample rate
    
    order : int
        The order of the filter
    """

    nyquist = 0.5 * fs
    cut = cutoff / nyquist
    b, a = butter(order, cut, btype='low')
    y = lfilter(b,a,data)
    return y

# Filter and detect peaks for each channel
def detect_wave_packet(channel_data):
    """
    Detects the peaks in a channel's data and returns the peak indices and the filtered signal.

    Parameters
    ----------
    channel_data : np.array
        The channel's data

    Returns
    -------
    peaks : np.array
        The peak indices

    filtered_signal : np.array
        The filtered signal
    """

    # Filter the channel data
    filtered_signal = butter_bandpass_filter(channel_data)

    # Find peaks in the filtered signal
    peaks, _ = find_peaks(filtered_signal, height=0.01, distance=100)

    return peaks, filtered_signal

DATA = np.genfromtxt('sample1.csv', delimiter=',', skip_header=1)
RECTIFIED = np.absolute(DATA[:,1])
CHANNELS = [np.absolute(DATA[:,i+1]) for i in range(3)]

data_cutoff = 0

r1 = butter_bandpass_filter(RECTIFIED, 50000, 58000)[data_cutoff:]
# r2 = butter_bandpass_filter(RECTIFIED, 25000, 40000)[data_cutoff:]

plt.plot(DATA[data_cutoff:,0], r1)

plt.show()
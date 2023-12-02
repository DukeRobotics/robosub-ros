import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from data_sim import gen_timeseries
import matplotlib as mpl
import time

mpl.rcParams['agg.path.chunksize'] = 100_000

LOWCUT = 34_500
HIGHCUT = 35_500
PING_FREQ = 35_000
BANDPASS_WIDTHS = [4_000, 2_000, 1000, 750, 500]
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

def main():
    # Filter and detect peaks for all channels
    # timeseries = gen_timeseries(np.array([10,10,0]))

    # Read data from csv file
    data_0 = np.genfromtxt('sample1.csv', delimiter=',', skip_header=1)

    # Use only the first 20% of the data
    # Data is of shape n by 4
    

    rectified = np.absolute(data_0[:,1])
    # peaks = []
    filtered_signals = []
    start = time.perf_counter()
    for i in range(5):
        #channel_peaks, filtered_signal = detect_wave_packet(data_0[:,i+1])
        #peaks.append(channel_peaks)
        filtered_signal = butter_bandpass_filter(rectified, PING_FREQ -  BANDPASS_WIDTHS[i]/2, PING_FREQ + BANDPASS_WIDTHS[i]/2) # do a bandpass of the data from channel 1
        filtered_signal = butter_lowpass_filter(np.absolute(filtered_signal), LOWPASS_CUTOFF) #arbitrarily chosen cutoff. 
        filtered_signals.append(filtered_signal)
        print(f"{i} done, {time.perf_counter() - start} ms elapsed")

    print([len(filtered_signals[i]) for i in range(5)])


    
    # peaks = []
    # filtered_signals = []
    # for channel_data in timeseries:
    #     channel_peaks, filtered_signal = detect_wave_packet(channel_data)
    #     peaks.append(channel_peaks)
    #     filtered_signals.append(filtered_signal)

    #print([len(peaks[i]) for i in range(1)])

    #print(peaks)

    # Plot the detected peaks against both the raw and filtered signals
    # 2x4 subplots

    _, ax = plt.subplots(3,2, figsize=(10,10))
    # plt.yscale("log")

    ax[0,0].plot(data_0[:,0], butter_lowpass_filter(rectified, LOWPASS_CUTOFF))

    for i in range(1,6):
        # Plot the raw signal
        

        # Plot the filtered signal
        ax[i//2,i%2].plot(data_0[:,0], filtered_signals[i-1])

        # Plot the detected peaksx
        #ax[i,1].plot(data_0[peaks[i],0], filtered_signals[i][peaks[i]], "x")

        ax[i//2,i%2].set_title(f"Bandwidth : {BANDPASS_WIDTHS[i-1]}")
        ax[i//2,i%2].set_xlabel("Time (s)")
        ax[i//2,i%2].set_ylabel("Amplitude")
        # ax[i,1].set_yscale("log")
        # ax[i,1].set_ylim(-100,100)

    
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()

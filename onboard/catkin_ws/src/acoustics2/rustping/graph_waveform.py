import array
import struct
from collections import namedtuple
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from scipy.signal import find_peaks, spectrogram
# from data_sim import gen_timeseries
import matplotlib as mpl
import time
from scipy.signal import hilbert
mpl.rcParams['agg.path.chunksize'] = 100_000

LOWCUT = 34_500
HIGHCUT = 35_500
PING_FREQ = 35_000
BANDPASS_WIDTHS = [4_000, 2_000, 1000, 750, 500]
BANDPASS_WIDTH = 1000
LOWPASS_CUTOFF = 6_000 # kinda arbitrarily chosen. In practice, cutoff for envelope detection should be square root of carrier freq (35 kHz) and message freq (not quite sure, chose 1 kHz?) TODO: Run fft on data and see what the frequency spectrum looks like

SAMPLE_RATE = 625_000

TYPE_DIGITAL = 0
TYPE_ANALOG = 1
expected_version = 0
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

def get_spike_starts(rectified_data, sample_rate):
    filtered = butter_bandpass_filter(rectified_data, 50000, 58000)
    envelope = np.abs(hilbert(filtered))
    all_peaks = envelope[find_peaks(envelope)[0]]
    print(len(all_peaks))
    mean = np.mean(all_peaks)
    stdev = np.std(all_peaks)
    thresh = mean + 5*stdev
    thresh_peaks = []
    sample = 0
    while sample < len(filtered):
        if envelope[sample] > thresh:
            while envelope[sample-1] < envelope[sample]:
                sample -= 1
            thresh_peaks.append(sample)
            sample += sample_rate
        sample += 1
    return filtered, np.array(thresh_peaks)


AnalogData = namedtuple('AnalogData', ('begin_time', 'sample_rate', 'downsample', 'num_samples', 'samples'))

def parse_analog(f):
    # Parse header
    identifier = f.read(8)
    if identifier != b"<SALEAE>":
        raise Exception("Not a saleae file")

    version, datatype = struct.unpack('=ii', f.read(8))

    if version != expected_version or datatype != TYPE_ANALOG:
        raise Exception("Unexpected data type: {}".format(datatype))

    # Parse analog-specific data
    begin_time, sample_rate, downsample, num_samples = struct.unpack('=dqqq', f.read(32))

    # Parse samples
    samples = array.array("f")
    samples.fromfile(f, num_samples)

    return AnalogData(begin_time, sample_rate, downsample, num_samples, samples)


if __name__ == '__main__':
    # filename = sys.argv[1]
    # if not filename:
    filename = 'octants_data/0/1bin/analog_2.bin'
    print("Opening " + filename)

    start_time = time.perf_counter()
    with open(filename, 'rb') as f:
        data = parse_analog(f)
    end_time = time.perf_counter()
    # print as ms
    print("Time to parse: " + str((end_time - start_time) * 1000) + "ms")

    # Print out all analog data
    print("Begin time: {}".format(data.begin_time))
    print("Sample rate: {}".format(data.sample_rate))
    print("Downsample: {}".format(data.downsample))
    print("Number of samples: {}".format(data.num_samples))

    print("  {0:>20} {1:>10}".format("Time", "Voltage"))
    
    filtered = butter_bandpass_filter(data.samples, 50000, 58000)[::10]
    # convolve for moving average
    conv = np.convolve(filtered, np.ones(50)/50, mode='valid')
    
    # d = get_spike_starts(conv, SAMPLE_RATE/50)
    # print(d)
    
    # print(len(data.samples))
    # plot waveform
    
    plt.plot(conv)
    # plt.scatter(d, np.zeros((len(d),)), color='red', marker='x')
    plt.show()

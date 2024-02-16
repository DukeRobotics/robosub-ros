import array
import struct
import sys
from collections import namedtuple
import time

TYPE_DIGITAL = 0
TYPE_ANALOG = 1
expected_version = 0

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
    filename = '/Users/ethanhorowitz/Desktop/new acoustics data/3/1bin/analog_0.bin'
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
    
    print(len(data.samples))

    # for idx, voltage in enumerate(data.samples):
    #     sample_num = idx * data.downsample
    #     time = data.begin_time + (float(sample_num) / data.sample_rate)
    #     print("  {0:>20.10f} {1:>10.3f}".format(time, voltage))
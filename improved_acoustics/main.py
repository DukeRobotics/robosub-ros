import cross_corr_fft as cc
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import acoustics_math as ac

filename = "/Users/reedchen/Downloads/625k_40k_80_30_186.csv"
filename = "/Users/reedchen/OneDrive - Duke University/Robotics/Data/matlab_custom_-2_7_4_.csv"
#data = ac.read_data(filename)
#data = np.array(data)
# print(type(data))
# print(data.shape)
# subset = data[1, 400:800]
# print(subset.shape)
#plt.plot(data[0])
#plt.show()

freq = 40000
guess = [-2,7,4]

processor = cc.AcousticProcessor(filename=filename, if_double=True, version=4, fs=625000, freq=freq, guess=guess,
                                 publish_counts=None, if_plot=True)
final_ccwh, valid_count, success_count = processor.run()
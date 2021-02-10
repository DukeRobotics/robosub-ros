import pandas
# import matplotlib.pyplot as plt
import math
import numpy as np


def dis(x, y, z, x1, y1, z1):
    return math.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)


sf = 625000  # sampling freq
pinger_frequency = 40000
vs = 1511.5  # velocity of sound
samples = math.floor(2.048 * 2 * sf)
noise = -60  # noise level in decibels

# pinger location
px = -2
py = 7
pz = 4

# cheap hydrophone location
space = .3
hp1 = [0, 0, 0]
hp2 = [space, 0, 0]
hp3 = [0, space, 0]
hp4 = [0, 0, space]

# expensive hydrophone location
space = 0.0115  # spacing between hydrophones
hp1 = [0, 0, 0]  # sqrt((px-x)^2+(py-y)^2+(pz-z)^2) = sqrt(69) = 8.307
hp2 = [0, -space, 0]  # sqrt(69.161) = 8.316
hp3 = [-space, 0, 0]  # sqrt(68.954) = 8.304
hp4 = [-space, -space, 0]  # sqrt(69.115) = 8.314
# in order from closest to pinger to farthest: h3, h1, h4, h2

dis1 = dis(px, py, pz, hp1[0], hp1[1], hp1[2])
dis2 = dis(px, py, pz, hp2[0], hp2[1], hp2[2])
dis3 = dis(px, py, pz, hp3[0], hp3[1], hp3[2])
dis4 = dis(px, py, pz, hp4[0], hp4[1], hp4[2])

# hydrophone channel data
ping1 = np.zeros(samples)
ping2 = np.zeros(samples)
ping3 = np.zeros(samples)
ping4 = np.zeros(samples)

# ping
s = np.arange(0, math.floor(0.004 * sf))
ping = 0.1 * np.sin((s / sf * pinger_frequency * 2 * math.pi))  # complex conjugate transpose (np.matrix.H)

# samples until ping occurs
buffer = 40000

ping1[math.ceil(dis1 / vs * sf) + buffer: math.ceil((dis1 / vs + 0.004) * sf) + buffer] = ping

ping2[math.ceil(dis2 / vs * sf) + buffer: math.ceil((dis2 / vs + 0.004) * sf) + buffer] = ping

ping3[math.ceil(dis3 / vs * sf) + buffer: math.ceil((dis3 / vs + 0.004) * sf) + buffer] = ping

ping4[math.ceil(dis4 / vs * sf) + buffer: math.ceil((dis4 / vs + 0.004) * sf) + buffer] = ping

ping1[math.ceil(dis1 / vs * sf + 2.048 * sf) + buffer:
      math.ceil((dis1 / vs + 0.004) * sf + 2.048 * sf) + buffer] = ping

ping2[math.ceil(dis2 / vs * sf + 2.048 * sf) + buffer:
      math.ceil((dis2 / vs + 0.004) * sf + 2.048 * sf) + buffer] = ping

ping3[math.ceil(dis3 / vs * sf + 2.048 * sf) + buffer:
      math.ceil((dis3 / vs + 0.004) * sf + 2.048 * sf) + buffer] = ping

ping4[math.ceil(dis4 / vs * sf + 2.048 * sf) + buffer:
      math.ceil((dis4 / vs + 0.004) * sf + 2.048 * sf) + buffer] = ping

for i in range(1, 5):
    mean = 0
    std = 0.01

    h1 = ping1 + np.random.normal(mean, std, samples)
    h2 = ping2 + np.random.normal(mean, std, samples)
    h3 = ping3 + np.random.normal(mean, std, samples)
    h4 = ping4 + np.random.normal(mean, std, samples)

    df = pandas.DataFrame({'Channel 0': h1, 'Channel 1': h2, 'Channel 2': h3, 'Channel 3': h4})
    df.to_csv(f'../matlab-expensive_-2_7_4_({i}).csv')

# plt.plot(h1)
# plt.plot(h2)
# plt.plot(h3)
# plt.plot(h4)
# plt.legend(['h1', 'h2', 'h3', 'h4'])
# plt.show()

import numpy as np
from sklearn.cluster import KMeans, SpectralClustering
import matplotlib.pyplot as plt
from matplotlib import colors

with open('training.csv', 'r') as f:
    data = f.read().split('\n')[1:]
    data = [x.split(',') for x in data if x]
    data = [[float(x) for x in y] for y in data]
    data = list(filter(lambda x: max(x[:-1]) < 0.5, data))
    x1 = p.array([x[0] for x in data])
    x2 = np.array([x[1] for x in data])
    x3 = np.array([x[2] for x in data])
    y = np.array([x[3] for x in data])

unique_angles = sorted(list(set(y)))
colors_range = colors.Normalize(vmin=min(unique_angles), vmax=max(unique_angles))

# separate points radially by x1, x2, and y
point_angles = np.arctan2(x2, x1)
point_rs = np.sqrt(x1**2 + x2**2)

# k-means on angle and y
model = KMeans(n_clusters=5)
X = np.array([point_angles, point_rs]).T
labels = model.fit_predict(X)

# graph it, marker by label, color by y

marker_types = ['x', 'o', 'v', 'D', '|', '>', 's', 'p', 'P', '*', 'h', 'H', 'D', 'd', '|', '_'][:5]
x1s = []
x2s = []
ys = []
for i in range(5):
    x1s.append(x1[labels == i])
    x2s.append(x2[labels == i])
    ys.append(y[labels == i])
    
for i in range(5):
    plt.scatter(x1s[i], x2s[i], c=y_colors, marker=marker_types[i])

plt.colorbar()
plt.show()

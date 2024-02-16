import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans, SpectralClustering
from scipy.spatial import ConvexHull

with open('training.csv', 'r') as f:
    data = f.read().split('\n')[1:]
    data = [x.split(',') for x in data if x]
    data = [[float(x) for x in y] for y in data]
    data = list(filter(lambda x: max(x[:-1]) < 0.5, data))
    x1 = np.array([x[0] for x in data])
    x2 = np.array([x[1] for x in data])
    x3 = np.array([x[2] for x in data])
    y = np.array([x[3] for x in data])

print(y)

marker_types = ['x', 'o', 'v', 'D', '|', '>', 's', 'p', 'P', '*', 'h', 'H', 'D', 'd', '|', '_'][:5]
unique_angles = sorted(list(set(y)))
colors = [unique_angles.index(x) for x in y]
    
model = SpectralClustering(n_clusters=4, affinity='nearest_neighbors',
                           assign_labels='kmeans')
X = np.array([x1, x2]).T
labels = model.fit_predict(X)

x1s = []
x2s = []
ys = []
for i in range(5):
    x1s.append(x1[labels == i])
    x2s.append(x2[labels == i])
    ys.append(y[labels == i])

# fig = plt.figure()
# ax = fig.add_subplot(121)

print(unique_angles)

for i in range(5):
    plt.scatter(x1s[i], x2s[i], c=[unique_angles.index(x) for x in ys[i]], marker=marker_types[i])

plt.colorbar()
    
# ax2 = fig.add_subplot(122)
# cmap = mpl.colors.ListedColormap(colors)
# cmap.set_over('0.25')
# cmap.set_under('0.75')

# bounds = sorted(list(set(y)))
# norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
# cb2 = mpl.colorbar.ColorbarBase(ax2, cmap=cmap,
#                                 norm=norm,
#                                 ticks=bounds,
#                                 spacing='proportional',
#                                 orientation='vertical')

#make colorbar



# all points are planar, calculate plane
# plane is ax + by + cz + d = 0
# x = np.array([x1, x2]).T
# A = np.c_[x, np.ones(x.shape[0])]
# C, _, _, _ = np.linalg.lstsq(A, x3, rcond=None)
# print(C)

# project points [x1, x2, x3] onto plane in new (x, y) space

# k-means x1, x2, and y
# kmeans = KMeans(n_clusters=5)
# y_pred = kmeans.fit_predict(np.array([x1, x2]).T)

# plt.scatter(x1, x2, c=y_pred)



# # spread colors across the angles


# fig = plt.figure()
# ax = fig.add_subplot(111)
# plt.scatter(x1, x2, c=colors, marker=label_markers)

# ax.scatter(centers[:, 0], centers[:, 1], c='red', marker='x')
# # color k-means areas
# plt.imshow(means.labels_.reshape(len(x1), 1), aspect='auto', cmap='viridis', alpha=0.25, extent=(min(x1), max(x1), min(x2), max(x2)))
    

    

plt.show()



# # graph 3d x1, x2, and y
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# # ax.scatter(x1, x2, x3, c=y)
# ax.set_xlabel('x1')
# ax.set_ylabel('x2')
# ax.set_zlabel('x3')
# # colorbar
# plt.colorbar(ax.scatter(x1, x2, x3, c=colors))

# # plot plane
# plane_x = np.linspace(min(x1), max(x1), 10)
# plane_y = np.linspace(min(x2), max(x2), 10)
# plane_x, plane_y = np.meshgrid(plane_x, plane_y)
# plane_z = C[0] * plane_x + C[1] * plane_y + C[2]
# # make plane gray
# ax.plot_surface(plane_x, plane_y, plane_z, alpha=0.25, color='gray')

# plt.show()
    
    
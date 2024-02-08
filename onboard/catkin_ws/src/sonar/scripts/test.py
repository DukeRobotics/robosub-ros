import numpy as np
import matplotlib.pyplot as plt
import os
from sklearn.cluster import DBSCAN
from sklearn.datasets import make_blobs

# Directory containing the .npy files
data_dir = r"C:\Users\willd\Robotics\robosub-ros\onboard\catkin_ws\src\sonar\scripts\sampleData2"

# Get a list of all .npy files in the directory
npy_files = [f for f in os.listdir(data_dir) if f.endswith('.npy')]

def stuff():
    for npy_file in npy_files:
        # Load the numpy array
        array = np.load(os.path.join(data_dir, npy_file))
        column_sums = np.sum(array, axis=0)
        average_column = np.sum(np.multiply(column_sums, np.arange(0, column_sums.size)))/np.sum(column_sums)

        row_sums = np.sum(array, axis=1)
        average_row = np.sum(np.multiply(row_sums, np.arange(0, row_sums.size)))/np.sum(row_sums)

        THRESHOLD = 90
        #apply threshold to array
        array[array < THRESHOLD] = 0

        #create plt plot
        fig, ax = plt.subplots()
        ax.imshow(array)
        ax.plot(average_column, average_row, 'ro')

        ax.axis('off')
        plt.show()

if __name__ == "__main__":
    array = np.load(data_dir + "\sonar_sweep_1706987047700.npy")

    THRESHOLD = 90
    array[array < THRESHOLD] = 0

    #convert values > 100 to list of points
    points = np.argwhere(array > 100)
    # print(points)

    db = DBSCAN(eps=3, min_samples=10).fit(points)
    labels = db.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    print("Number of clusters: ", n_clusters_)
    
    t_array = array.T
    # np.savetxt("sonar2.csv", t_array, fmt='%d', delimiter=',')

    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

    plt.figure(figsize=(14, 2))
    plt.imshow(array, cmap='viridis', aspect='auto')
    plt.colorbar(label='Array Values')
    plt.title('Visualization of the Original Array')
    plt.xlabel('Column Index')
    plt.ylabel('Row Index')

    for k, col in zip(unique_labels, colors):
        if k == -1:
            continue

        class_member_mask = (labels == k)

        xy = points[class_member_mask]
        plt.plot(xy[:, 1], xy[:, 0], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)

    plt.show()
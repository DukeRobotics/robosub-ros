import numpy as np
import matplotlib.pyplot as plt
import os
from sklearn.cluster import DBSCAN
import time
from sklearn.linear_model import LinearRegression
import math

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


VALUE_THRESHOLD = 95

if __name__ == "__main__":
    for npy_file in npy_files:
        # Load the numpy array
        array = np.load(os.path.join(data_dir, npy_file))

        # Set up the plot
        plt.figure(figsize=(14, 2))
        plt.imshow(array, cmap='viridis', aspect='auto')
        plt.colorbar(label='Array Values')
        plt.title('Visualization of the Original Array')
        plt.xlabel('Column Index')
        plt.ylabel('Row Index')

        # Start timer
        start_time = time.time()

        # Convert values > VALUE_THRESHOLD to list of points
        points = np.argwhere(array > VALUE_THRESHOLD)

        # Perform DBSCAN clustering
        db = DBSCAN(eps=3, min_samples=10).fit(points)
        labels = db.labels_

        # Get cluster with the most points
        unique_labels = set(labels)
        cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}
        print(cluster_counts)

        # Get the points of the largest cluster
        largest_cluster_label = max(cluster_counts, key=cluster_counts.get)
        class_member_mask = (labels == largest_cluster_label)
        buoy_points = points[class_member_mask]

        # Get the average column index of the largest cluster
        average_column_index = np.mean(buoy_points[:, 1])
        print(f"average column index: {average_column_index}")

        # Get the X and Y of the buoy points
        X = buoy_points[:, 0].reshape(-1, 1)
        Y = buoy_points[:, 1]

        # Create and fit the model
        linreg_sklearn = LinearRegression()
        linreg_sklearn.fit(X, Y)

        # Extract the slope (coefficient) and intercept
        slope_sklearn = linreg_sklearn.coef_[0]
        intercept_sklearn = linreg_sklearn.intercept_

        x_vals_plot = np.arange(array.shape[0])  # Row indices
        y_vals_plot = intercept_sklearn + slope_sklearn * x_vals_plot

        # Calculate the angle in radians
        angle = math.atan(slope_sklearn)
        angle = math.degrees(angle)

        print(f"angle: {angle}")

        print(f"slope: {slope_sklearn}, intercept: {intercept_sklearn}")

        plt.plot(y_vals_plot, x_vals_plot, 'r--', label=f'Line: y = {slope_sklearn:.2f}x + {intercept_sklearn:.2f}')
        plt.scatter(average_column_index, array.shape[0]/2, color='blue', s=50, label='Center Point')

        # plt.plot(buoy_points[:, 1], buoy_points[:, 0], 'o', markerfacecolor='r', markeredgecolor='k', markersize=6)

        plt.show()
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor, LinearRegression
import time as t
import os

def createImage(img, start_angle=0, center_angle=90, speed=1):
    """ create a cartesian top down image from a polar image with
    x coordinates as the distance away from the sonar and
    y coordinates as angles (in gradians)
    
    Args:
        path (string): path of the image
        start_angle (int): angle that the scan starts at
        center_angle (int): angle that the program should center the scan to
        speed (int): speed multiplier (also decreases resolution by same value)

    Returns:
        ndarray: image
    """
    step_x = 1*speed; step_y = 1*speed
    
    start = t.time()

    angle = img.shape[0]
    radius = img.shape[1]

    center = start_angle + angle // 2 # find center of the scan arc

    width = 2*radius + 1 #TODO limit size
    height = radius + 1

    polarImg = np.zeros((height, width))

    x_ax = np.linspace(-radius, radius, width, dtype=np.int32)
    y_ax = np.linspace(0, radius, height, dtype=np.int32)
    xx, yy = np.meshgrid(x_ax, y_ax)

    theta = np.rad2deg(np.arctan2(yy, xx))*200/180
    r = np.sqrt(xx**2 + yy**2)

    np.round(theta)
    np.round(r)
    
    theta = theta.astype(np.int32)
    r = r.astype(np.int32)

    for x in range(x_ax[0], x_ax[-1]+1, step_x):
        for y in range(y_ax[0], y_ax[-1]+1, step_y):
            theta_pt = theta[y][x + radius] - int(center_angle*200/180 - center) # shift angles to center the to center_angle
            r_pt = r[y][x + radius] # x + radius to convert x values into index values

            if (theta_pt < angle and theta_pt > 0 and r_pt < radius):
                polarImg[radius - y][radius - x] = img[theta_pt, r_pt] # radius - y flips to face the scan upward
            else:
                polarImg[radius - y][radius - x] = 0
    print("Total creation time: ", t.time() - start)

    return polarImg.astype(np.uint8)

def find_longest_cluster(image, threshold=125):
    """ find cluster with longest diagonal of the bounding box
        this should be the wall of the pool
    
    Args:
        image (ndarray): image to process
        threshold (int): threshold value to filter image

    Returns:
        ndarray: longest cluster
    """

    # Convert values > VALUE_THRESHOLD to list of points
    points = np.argwhere(image > threshold)

    # Perform DBSCAN clustering
    db = DBSCAN(eps=100, min_samples=10).fit(points)
    labels = db.labels_

    # Get clusters
    unique_labels = set(labels)
 
    # Find longest cluster
    longest_cluster = None

    for k in unique_labels:
        if k != -1:
            class_member_mask = (labels == k)
            buoy_points = points[class_member_mask]

            # find length of diagonal of bounding box of each cluster
            min_row = np.min(buoy_points[:, 0])
            max_row = np.max(buoy_points[:, 0])
            min_col = np.min(buoy_points[:, 1])
            max_col = np.max(buoy_points[:, 1])

            row_diff = max_row - min_row
            col_diff = max_col - min_col
            dist = np.square(row_diff) + np.square(col_diff)

            # find cluster that has longest diagonal
            if longest_cluster is None or dist > longest_cluster[0]:
                longest_cluster = (dist, buoy_points)

    return longest_cluster[1]

def add_linear_regression(image, cluster, speed=10):
    """ plots linear regression (ransac) of longest cluster and finds slope
    
    Args:
        image (ndarray): sonar data transformed into cartesian form
        cluster (ndarray): points of longest cluster

    Returns:
        tuple: (x, y) point of the center of the scanned wall
    """ 
       
    # apply linear regression to the cluster
    X = cluster[:, 0].reshape(-1, 1)
    Y = cluster[:, 1]
    ransac = RANSACRegressor(LinearRegression())
    ransac.fit(X, Y)

    # Get the slope (coefficient) and intercept
    slope = ransac.estimator_.coef_[0]
    intercept = ransac.estimator_.intercept_

    # # Plot linear regression line
    # x_vals_plot = np.arange(image.shape[1])
    # y_vals_plot = intercept + slope * x_vals_plot
    # plt.plot(y_vals_plot, x_vals_plot, 'r', linewidth=4, label=f'Line: y = {slope:.2f}x + {intercept:.2f}')

    # find indices of the first and last inlier
    lower_bound = -1
    upper_bound = -1
    inlier_mask = ransac.inlier_mask_

    for i in range(0, len(inlier_mask), speed):
        if inlier_mask[i] and lower_bound == -1:
            lower_bound = i
        elif inlier_mask[i] and lower_bound != -1:
            upper_bound = i

    # get region of x values that are inliers
    lower_x = cluster[lower_bound][0]
    upper_x = cluster[upper_bound][0]

    col_center_of_wall = (lower_x + upper_x)/2
    row_center_of_wall = ransac.predict([[col_center_of_wall]])[0]

    print(inlier_mask.shape)
    print(f"bounds = {lower_x}, {upper_x}")
    print(f"center = ({col_center_of_wall}, {row_center_of_wall})")

    return (col_center_of_wall, row_center_of_wall)

def processImage(image, image_conversion_speed=1, linear_regression_speed=10):
    """ transform image from polar to cartesian, then finds center of the pool wall in the image
    
    Args:
        image (ndarray): polar image from sonar
        cluster (ndarray): points of longest cluster

    """
    start = t.time()

    sonar_img = createImage(image, speed=image_conversion_speed)
    cluster = find_longest_cluster(sonar_img)
    center_of_wall = add_linear_regression(sonar_img, cluster, speed=linear_regression_speed)

    # # Set up the plot
    plt.imshow(sonar_img, cmap='viridis', aspect='auto')
    plt.colorbar(label='Array Values')
    plt.title('Visualization of the Original Array')
    plt.xlabel('Column Index')
    plt.ylabel('Row Index')
    plt.xlim(0, 2400)
    plt.ylim(1200, 0)

    # plot longest diagonal cluster
    plt.plot(cluster[:, 1], cluster[:, 0], 'o', markerfacecolor='g', markeredgecolor='k', markersize=6)

    # plot location of the center of the line
    plt.scatter(center_of_wall[1], center_of_wall[0], color='w', s=1000, label='Point')

    print(f"total processing took {t.time() - start}")

    plt.show()

def main():
    # Directory containing the .npy files
    # data_dir = r"onboard\catkin_ws\src\sonar\sonar_cartesian_data"
    data_dir = r"onboard\catkin_ws\src\sonar\sonar_wall_sample_data"

    # Get a list of all .npy files in the directory
    npy_files = [f for f in os.listdir(data_dir) if f.endswith('.npy')]

    for npy_file in npy_files:
        path = os.path.join(data_dir, npy_file)
        print(path)

        image = np.load(path)

        processImage(image, image_conversion_speed=1, linear_regression_speed=10)
        
if __name__ == '__main__':
    main()
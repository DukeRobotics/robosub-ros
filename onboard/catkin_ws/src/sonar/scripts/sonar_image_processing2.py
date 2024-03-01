import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import time as t
from sklearn.cluster import DBSCAN
from shapely.geometry import MultiPoint
import os
from scipy import stats
import csv

def getImage(path):
    """ turn jpeg into numpy array
    
    Args:
        path (string): path of the image

    Returns:
        ndarray: image
    """
    if (path.endswith(".npy")):
        return np.load(path)
    
    img = Image.open(path)
    npImg = np.asarray(img)
    
    return npImg


def createImage(img, start_angle=0, center_angle=90, speed="slow"):
    """ create a cartesian top down image from a polar image with
    x coordinates as the distance away from the sonar and
    y coordinates as angles (in gradians)
    
    Args:
        path (string): path of the image
        start_angle (int): angle that the scan starts at
        center_angle (int): angle that the program should center the scan to

    Returns:
        ndarray: image
    """
    step_x = 1; step_y = 1
    if speed == "fast":
        step_x = 2; step_y = 2

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


def addContours(image, lower_bound=(0, 127, 0), upper_bound=(255, 255, 255), kernel=(17, 17), area_threshold=5000, line_color=(0, 0, 255)):
    """ gaussian blurs the image then adds contours
    
    Args:
        image (ndarray): image to process
        lower_bound (tuple): lower bound when thresholding the image to find contour regions
        upper_bound (tuple): upper bound when thresholding the image to find contour regions
        kernel (tuple): kernel to use when applying gaussian blur
        area_threshold (int or float): only shows contours that are bigger than this value
        line_color (tuple): BGR value for cv2 to draw contours and centroid dot
    """
    mask = cv2.inRange(image, lower_bound, upper_bound)

    blurred_img = cv2.GaussianBlur(mask, kernel, 0)

    contours, hierarchy = cv2.findContours(blurred_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shapes = []
    print("Num contours", len(contours))
    for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        if moments['m00'] > area_threshold:
            shapes.append(contours[i])
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(image, (cx, cy), 8, line_color, -1)
            cv2.drawContours(image, contours, i, line_color, 2)
    printCirularity(shapes)
    return image

def findClusters(image, threshold=139):

    # Convert values > VALUE_THRESHOLD to list of points
    
    points = np.argwhere(image > threshold)

    # Set up the plot
    plt.imshow(image, cmap='viridis', aspect='auto')
    plt.colorbar(label='Array Values')
    plt.title('Visualization of the Original Array')
    plt.xlabel('Column Index')
    plt.ylabel('Row Index')
    # Perform DBSCAN clustering
    db = DBSCAN(eps=15, min_samples=10).fit(points)
    labels = db.labels_

    # Get cluster with the most points
    unique_labels = set(labels)
    # cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}
    # print(len(cluster_counts))

    # Get the points of the largest cluster
    # largest_cluster_label = max(cluster_counts, key=cluster_counts.get)
    # class_member_mask = (labels == largest_cluster_label)
    # buoy_points = points[class_member_mask]

    clusters = []

    for k in unique_labels:
        if k != -1:
            a_thresh = 6000
            c_thresh = 0.5

            class_member_mask = (labels == k)
            buoy_points = points[class_member_mask]

            average_row_index = np.mean(buoy_points[:, 0])
            # print(average_row_index)

            # get cluster info
            cluster = MultiPoint(buoy_points)

            cluster_shape = cluster.convex_hull
            perimeter = cluster_shape.length
            area = cluster_shape.area
            circularity = 4*np.pi*area/perimeter**2

            if (area > a_thresh and circularity < c_thresh):

                # get line of best fit for cluster
                slope, intercept, r_value, p_value, std_err = stats.linregress(buoy_points[:, 0], buoy_points[:, 1])
                x_regression = np.linspace(np.min(buoy_points[:, 0]), np.max(buoy_points[:, 0]), 1000)
                y_regression = slope * x_regression + intercept
                # plt.plot(y_regression, x_regression, color='red', label='Linear Regression Line')
                # TODO - slope metric doesn't work, density of cluster is messing it up
                # TODO - find regression slope of just 1 of each X value

                # scale the area threshold if the target is too close to the sonar
                if average_row_index > (image.shape[0] - 300):
                    scaling = (image.shape[0] - average_row_index)/300
                    a_thresh = a_thresh*scaling

                dont_append = False

                # label and count num clusters that meet conditions
                for i in range(len(clusters)):
                    if ((inRange(area, clusters[i][0].area, 0.3)
                    and inRange(perimeter, clusters[i][0].length, 0.3)
                    and average_row_index > clusters[i][2])):
                    # or (inRange(slope, clusters[i][3], 0.3))):
                        # found echo, replace with cluster closer to sonar
                        clusters[i] = [cluster_shape, buoy_points, average_row_index, slope]
                        dont_append = True
                if not dont_append:
                    clusters.append([cluster_shape, buoy_points, average_row_index, slope])


    colors = ['b', 'g', 'r', 'c', 'm', 'y']
    count = 0

    for cluster in clusters:
        plt.plot(cluster[1][:, 1], cluster[1][:, 0], 'o', markerfacecolor=colors[count%6], markeredgecolor='k', markersize=6)
        perimeter = cluster[0].length
        area = cluster[0].area
        circularity = 4*np.pi*area/perimeter**2
        print(count%6, perimeter, area, circularity, cluster[3])
        count += 1

    # Get the average column index of the largest cluster
    # average_column_index = np.mean(buoy_points[:, 1])
    # print(average_column_index)
    # print(f"average column index: {average_column_index}")
    # plt.scatter(average_column_index, image.shape[0]/2, color='blue', s=50, label='Center Point')

    plt.show()

    return count

def inRange(input, src, scaling):
    upper = src + scaling*src
    lower = src - scaling*src
    if input < upper and input > lower:
        return True
    else:
        return False

def printCirularity(contours):

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        if perimeter == 0:
            break
        circularity = 4*np.pi*(area/perimeter*perimeter)
        print("Circularity: ", circularity)


def main():
    # Directory containing the .npy files
    data_dir = r"C:\Users\pzhen\VSCodeProjects\robosub-ros\onboard\catkin_ws\src\sonar\sampleData"

    # Get a list of all .npy files in the directory
    npy_files = [f for f in os.listdir(data_dir) if f.endswith('.npy')]

    # # Clear the csv file
    # with open('analysis.csv', 'w') as file:
    #     file.write("")
    #     file.write(data_dir)
    #     file.write("\n")
    #     file.write("Time, Cluster Count, Path\n")

    # Loop through all data to find num_clusters found
    with open('analysis.csv', 'a') as file:
        # for npy_file in npy_files:
        #     path = os.path.join(data_dir, npy_file)
            
            path = os.path.join(data_dir, "sonar_sweep_14.npy") #---------
            sonar_img = np.load(path)
            print(path)

            start = t.time()

            sonar_img_polar = createImage(sonar_img, speed="slow")
            sonar_img_polar = cv2.cvtColor(sonar_img_polar.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            sonar_img_polar = cv2.applyColorMap(sonar_img_polar, cv2.COLORMAP_VIRIDIS)

            num_clusters = findClusters(sonar_img_polar)
            print("Total processing time: ", t.time() - start)

            # idx = path.index("sonar\\sampleData")
            # file.write(str(round(t.time() - start, 2)) + ", " + str(num_clusters) + ", " + path[idx+6:] + "\n")

            # addContours(sonar_img_polar)
            resized_img = cv2.resize(sonar_img_polar, (sonar_img_polar.shape[1] // 2, sonar_img_polar.shape[0] // 2))
            
            # print(sonar_img_polar.shape)

            # cv2.imshow('sonar image', resized_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

    file.close()


if __name__ == '__main__':
    main()
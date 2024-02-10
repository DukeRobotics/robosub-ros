import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import time as t
from sklearn.cluster import DBSCAN
from shapely.geometry import MultiPoint
import os

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

    msehgrid_start = t.time()
    x_ax = np.linspace(-radius, radius, width, dtype=np.int32)
    y_ax = np.linspace(0, radius, height, dtype=np.int32)
    xx, yy = np.meshgrid(x_ax, y_ax)
    print("create meshgrid time: ", t.time() - msehgrid_start)

    theta = np.rad2deg(np.arctan2(yy, xx))*200/180
    r = np.sqrt(xx**2 + yy**2)

    np.round(theta)
    np.round(r)
    
    theta = theta.astype(np.int32)
    r = r.astype(np.int32)

    create_img_start = t.time()
    # for x in x_ax:
    #     for y in y_ax:
    for x in range(x_ax[0], x_ax[-1]+1, step_x):
        for y in range(y_ax[0], y_ax[-1]+1, step_y):
            theta_pt = theta[y][x + radius] - int(center_angle*200/180 - center) # shift angles to center the to center_angle
            r_pt = r[y][x + radius] # x + radius to convert x values into index values

            if (theta_pt < angle and theta_pt > 0 and r_pt < radius):
                polarImg[radius - y][radius - x] = img[theta_pt, r_pt] # radius - y flips to face the scan upward
            else:
                polarImg[radius - y][radius - x] = 0
    print("create image time: ", t.time() - create_img_start)
    print("total time: ", t.time() - start)
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
    cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}
    # print(len(cluster_counts))

    # Get the points of the largest cluster
    # largest_cluster_label = max(cluster_counts, key=cluster_counts.get)
    # class_member_mask = (labels == largest_cluster_label)
    # buoy_points = points[class_member_mask]
    
    colors = ['b', 'g', 'r', 'c', 'm', 'y']

    for k in unique_labels:
        if k != -1:
            class_member_mask = (labels == k)
            buoy_points = points[class_member_mask]

            # get cluster info
            cluster_shape = MultiPoint(buoy_points).convex_hull
            perimeter = cluster_shape.length
            area = cluster_shape.area
            circularity = 4*np.pi*area/perimeter**2

            if (area > 7500 and circularity < 0.375):
                plt.plot(buoy_points[:, 1], buoy_points[:, 0], 'o', markerfacecolor=colors[k%6], markeredgecolor='k', markersize=6)
                print(k, colors[k%6], perimeter, area, circularity)


    #Get the average column index of the largest cluster
    # average_column_index = np.mean(buoy_points[:, 1])
    # print(f"average column index: {average_column_index}")
    # plt.scatter(average_column_index, image.shape[0]/2, color='blue', s=50, label='Center Point')

    # plt.plot(buoy_points[:, 1], buoy_points[:, 0], 'o', markerfacecolor='r', markeredgecolor='k', markersize=6)

    plt.show()


def printCirularity(contours):

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        if perimeter == 0:
            break
        circularity = 4*np.pi*(area/perimeter*perimeter)
        print("Circularity: ", circularity)


def main():
    start = t.time()

    # Directory containing the .npy files
    data_dir = r"C:\Users\pzhen\VSCodeProjects\robosub-ros\onboard\catkin_ws\src\sonar\sampleData"

    # Get a list of all .npy files in the directory
    npy_files = [f for f in os.listdir(data_dir) if f.endswith('.npy')]

    for npy_file in npy_files:
        path = os.path.join(data_dir, npy_file)
        sonar_img = np.load(path)
        print(path)

        # print(sonar_img)

        sonar_img_polar = createImage(sonar_img, speed="Good")
        sonar_img_polar = cv2.cvtColor(sonar_img_polar.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        sonar_img_polar = cv2.applyColorMap(sonar_img_polar, cv2.COLORMAP_VIRIDIS)

        findClusters(sonar_img_polar)
        # addContours(sonar_img_polar)
        # resized_img = cv2.resize(sonar_img_polar, (sonar_img_polar.shape[1] // 2, sonar_img_polar.shape[0] // 2))
        
        # # print(sonar_img_polar.shape)

        # cv2.imshow('sonar image', resized_img)
        print("total time for image: ", t.time() - start)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
   

if __name__ == '__main__':
    main()
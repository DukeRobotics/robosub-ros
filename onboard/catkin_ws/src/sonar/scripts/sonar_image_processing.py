import cv2
import matplotlib.pyplot as plt
import numpy as np
from decode_ping_python_ping360 import get_bin_file_parser
import os
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
import math
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

SONAR_IMAGE_WIDTH = 16
SONAR_IMAGE_HEIGHT = 2


def build_color_sonar_image_from_int_array(int_array, npy_save_path=None, jpeg_save_path=None):
    """ Build a sonar image from a list of data messages

    Args:
        data_list (List): List of data messages from either the Sonar device
                        or from a .bin file
        npy_save_path (str, optional): Path to save the sonar image as a
                                    .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a
                                    .jpeg file. Defaults to None.

    Returns:
        ndarray: Sonar image from the scan
    """

    sonar_img = cv2.cvtColor(int_array.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    sonar_img = cv2.applyColorMap(sonar_img, cv2.COLORMAP_VIRIDIS)
    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    return sonar_img


def find_center_point_and_angle(array, threshold, eps, min_samples, get_plot=True):
    """ Find the center point and angle of the largest cluster in the array

    Args:
        array (ndarray): The sonar image array
        threshold (int): The threshold to apply to the array
        eps (float): The maximum distance between two samples for one to be
                    considered as in the neighborhood of the other
        min_samples (int): The number of samples in a neighborhood for a point
                        to be considered as a core point
        get_plot (bool, optional): Whether to return the plot of the results.
                                Defaults to True.

    Returns:
        int: The average column index of the largest cluster in the array
        float: The angle of the largest cluster in the array
    """

    # Set up the plot
    if get_plot:
        plt.figure(figsize=(SONAR_IMAGE_WIDTH, SONAR_IMAGE_HEIGHT))
        plt.imshow(array, cmap='viridis', aspect='auto')

    # Convert values > VALUE_THRESHOLD to list of points
    points = np.argwhere(array > threshold)
    if points.size == 0:
        return None, None, None

    # Perform DBSCAN clustering
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_

    # Get cluster with the most points
    unique_labels = set(labels)
    cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}

    if cluster_counts == {}:
        return None, None, None

    # Get the points of the largest cluster and calculate the average column index
    largest_cluster_label = max(cluster_counts, key=cluster_counts.get)
    class_member_mask = (labels == largest_cluster_label)
    max_clust_points = points[class_member_mask]
    average_column_index = np.mean(max_clust_points[:, 1])

    # Get the X and Y of the buoy points and calc linear regression
    X = max_clust_points[:, 0].reshape(-1, 1)
    Y = max_clust_points[:, 1]
    linreg_sklearn = LinearRegression()
    linreg_sklearn.fit(X, Y)

    # Extract the slope and calculate the angle
    slope_sklearn = linreg_sklearn.coef_[0]
    intercept_sklearn = linreg_sklearn.intercept_
    angle = math.atan(slope_sklearn)
    angle = math.degrees(angle)

    # Plot the results
    if get_plot:
        x_vals_plot = np.arange(array.shape[0])  # Row indices
        y_vals_plot = intercept_sklearn + slope_sklearn * x_vals_plot
        plt.plot(y_vals_plot, x_vals_plot, 'r', linewidth=4,
                 label=f'Line: y = {slope_sklearn:.2f}x + {intercept_sklearn:.2f}')
        plt.scatter(average_column_index, array.shape[0]/2, color='k', s=150, zorder=3, label='Center Point')

        plt.xticks([])
        plt.yticks([])
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)

        fig = plt.gcf()
        canvas = FigureCanvas(fig)
        canvas.draw()

        image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return average_column_index, angle, image

    return average_column_index, angle, array


def build_sonar_img_from_log_file(filename, start_index=49, end_index=149):
    """ Build a sonar image from a .bin file

    Args:
        filename (str): Path to the .bin file
        start_index (int, optional): The index to start building the sonar image.
                                    Defaults to 49.
        end_index (int, optional): The index to stop building the sonar image.
                                Defaults to 149.

    Returns:
        ndarray: Sonar image from the scan

    """
    assert filename.endswith('.bin'), 'filename must be a .bin file'

    parser = get_bin_file_parser(filename)

    data_list = []
    for index, (timestamp, decoded_message) in enumerate(parser):
        if index >= start_index and index <= end_index:
            data_list.append(decoded_message.data)

    jpeg_save_path = os.path.join(os.path.dirname(__file__),
                                  'sampleData', 'Sonar_Image.jpeg')

    sonar_img = build_sonar_image(data_list, display_results=True,
                                  jpeg_save_path=jpeg_save_path)
    return sonar_img


def build_sonar_image(data_list, display_results=False,
                      npy_save_path=None, jpeg_save_path=None):
    """ Build a sonar image from a list of data messages

    Args:
        data_list (List): List of data messages from either the Sonar device
                          or from a .bin file
        display_results (bool, optional): Whether to display the resulting
                                          sonar image. Defaults to False.
        npy_save_path (str, optional): Path to save the sonar image as a
                                       .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a
                                       .jpeg file. Defaults to None.

    Returns:
        ndarray: Sonar image from the scan
    """

    sonar_img = None
    for data in data_list:
        split_bytes = [data[i:i+1] for i in range(len(data))]
        split_bytes = split_bytes[100:]

        byte_from_int = int.from_bytes(split_bytes[0], "big")
        intarray = np.array([byte_from_int])

        for i in range(len(split_bytes) - 1):
            byte_from_int = int.from_bytes(split_bytes[i+1], "big")
            intarray = np.append(intarray, [byte_from_int])

        if sonar_img is None:
            sonar_img = np.asarray(intarray)
        else:
            sonar_img = np.vstack((sonar_img, intarray))

    sonar_img = sonar_img.astype(np.uint8)

    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    if display_results:
        cv2.imshow("sonar_img", sonar_img)
        cv2.waitKey(0)

    return sonar_img

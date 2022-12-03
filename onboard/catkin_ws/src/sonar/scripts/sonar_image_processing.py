import cv2
import matplotlib.pyplot as plt
import numpy as np
import math


def scan_and_build_sonar_image(sonar,
                               display_results=False,
                               npy_save_path=None,
                               jpeg_save_path=None):
    """ Execute a sweep with the sonar device and then build a sonar image out of the results

    Args:
        sonar (Sonar): Sonar device being used
        range_start (int, optional): Angle to start scanning in gradians. Defaults to 100.
        range_end (int, optional): Angle to stop scanning in gradians. Defaults to 300.
        display_results (bool, optional): Whether to display the resulting sonar image. Defaults to False.
        npy_save_path (str, optional): Path to save the sonar image as a .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a .jpeg file. Defaults to None.

    Returns:
        ndarray: Sonar image from the scan
    """
    data_list = sonar.get_sweep()
    sonar_img = build_sonar_image(data_list, display_results, npy_save_path, jpeg_save_path)
    return sonar_img


def build_sonar_image(data_list, display_results=False, npy_save_path=None, jpeg_save_path=None):
    """ Build a sonar image from a list of data messages

    Args:
        data_list (List): List of data messages from either the Sonar device or from a .bin file
        display_results (bool, optional): Whether to display the resulting sonar image. Defaults to False.
        npy_save_path (str, optional): Path to save the sonar image as a .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a .jpeg file. Defaults to None.

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

    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    # TODO: test if this works
    if display_results:
        cv2.imshow("sonar_img", sonar_img)
        cv2.waitKey(0)

    return sonar_img


def find_gate_posts(img, display_results=False):
    """ Find gate posts from a sonar scan image

    Args:
        img (ndarray): Sonar image
        display_results (bool, optional): Whether to display the results. Defaults to False.

    Returns:
        List[Tuple]: List of of Tuples, where each tuple contains the (x,y) for a detected gate post in the image
    """

    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image, 5)  # Removes salt and pepper noise

    mask = mask_sonar_image(cm_image, display_results)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cm_circles = sorted(cm_circles, key=cv2.contourArea, reverse=True)
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 200), cm_circles))
    cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cm_circles))

    if len(cm_circles) < 1:
        print("Not enough circles found")
        return None

    filtered_circles = cm_circles[0:2]

    circle_positions = []
    for circle in filtered_circles:  # find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX, cY))

    if display_results:
        cv2.drawContours(cm_copy_image, filtered_circles, -1, (0, 255, 0), 2)
        cv2.imshow("found_gate_posts", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def find_bouy(img, display_results=False):
    """ Find buoys from a sonar scan image

    Args:
        img (ndarray): Sonar image
        display_results (bool, optional): Whether to display the results. Defaults to False.

    Returns:
        List[Tuple]: List of of Tuples, where each tuple contains the (x,y) for a detected buoy in the image
    """

    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image, 5)  # Removes salt and pepper noise

    mask = mask_sonar_image(cm_image, display_results)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 100), cm_circles))

    cm_circles = sorted(cm_circles,
                        key=lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x))),
                        reverse=True)

    filtered_circles = cm_circles[0:1]

    circle_positions = []
    for circle in filtered_circles:  # Find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX, cY))

    if display_results:
        cv2.drawContours(cm_copy_image, filtered_circles, -1, (0, 255, 0), 2)
        cv2.imshow("found_buoys", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def mask_sonar_image(img, display_results=False):
    """ Get mask of potential objects in a sonar image

    Args:
        img (ndarray): Image to mask
        display_results (bool, optional): Whether to display the results. Defaults to False.

    Returns:
        ndarray: Mask image
    """
    # TODO: this should probably be done with hue
    lower_color_bounds = (40, 80, 0)  # Filter out lower values (ie blue)
    upper_color_bounds = (230, 250, 255)  # Filter out too high values
    mask = cv2.inRange(img, lower_color_bounds, upper_color_bounds)

    if display_results:
        cv2.imshow("mask", mask)
        cv2.waitKey(0)

    return mask


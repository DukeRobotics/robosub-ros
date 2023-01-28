import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from decode_ping_python_ping360 import get_bin_file_parser
import os


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
    data_list = sonar.get_sweep(100, 300)
    sonar_img = build_sonar_image(data_list, display_results, npy_save_path, jpeg_save_path)
    return sonar_img


def build_sonar_img_from_log_file(filename, start_index=49, end_index=149):
    """ Builds a sonar image from a log file """
    assert filename.endswith('.bin'), 'filename must be a .bin file'

    parser = get_bin_file_parser(filename)

    data_list = []
    for index, (timestamp, decoded_message) in enumerate(parser):
        if index >= start_index and index <= end_index:
            data_list.append(decoded_message.data)

    JPEG_SAVE_PATH = os.path.join(os.path.dirname(__file__), 'sampleData', 'Sonar_Image.jpeg')

    sonar_img = build_sonar_image(data_list, display_results=True, jpeg_save_path=JPEG_SAVE_PATH)
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

    sonar_img = sonar_img.astype(np.uint8)

    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    if display_results:
        cv2.imshow("sonar_img", sonar_img)
        cv2.waitKey(0)

    return sonar_img


def arcity(c):
    t = tuple(c[c[:, :, 1].argmin()][0])
    b = tuple(c[c[:, :, 1].argmax()][0])
    l = tuple(c[c[:, :, 0].argmin()][0])
    r = tuple(c[c[:, :, 0].argmax()][0])
    bob = [t, b, l, r]

    max = -1
    for i in bob:
        for j in bob:
            if i == j:
                continue
            dist = ((i[0] - j[0])**2 + (i[1] - j[1])**2)**0.5
            if dist > max:
                max = dist

    ac = cv2.arcLength(c, True) / 2.0
    return max / ac


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

    kernel = np.ones((5, 5), np.uint8)

    # cm_image = cv2.erode(cm_image, kernel, iterations=1)
    kernel = np.ones((5, 5), np.uint8)
    cm_image = cv2.dilate(cm_image, kernel, iterations=3)
    kernel = np.ones((4, 4), np.uint8)
    cm_image = cv2.erode(cm_image, kernel, iterations=1)

    cm_image = cv2.medianBlur(cm_image, 5)  # Removes salt and pepper noise

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)

    mask = mask_sonar_image(cm_image, display_results)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 200 and cv2.contourArea(x) < 5000), cm_circles))
    cm_circles = sorted(cm_circles,
                        key=lambda x: (arcity(x)),
                        reverse=False)

    cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) > 2.5), cm_circles))

    if len(cm_circles) < 1:
        print("Not enough circles found")
        return None

    filtered_circles = cm_circles[0:1]

    circle_positions = []
    for circle in filtered_circles:  # find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX, cY, arcity(circle), cv2.arcLength(circle, True)**2/(4*math.pi*cv2.contourArea(circle))   ))

    if display_results:
        cv2.drawContours(cm_copy_image, filtered_circles, -1, (0, 255, 0), 2)
        cv2.imshow("found_gate_posts", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def find_buoy(img, display_results=False):
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
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 250), cm_circles))

    cm_circles = sorted(cm_circles,
                        key=lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x))),
                        reverse=False)

    filtered_circles = cm_circles[0:1]

    circle_positions = []
    for circle in filtered_circles:  # Find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX, cY, (cv2.arcLength(circle, True)**2/(4*math.pi*cv2.contourArea(circle))),
                                 cv2.contourArea(circle)))

    if display_results:
        cv2.drawContours(cm_copy_image, filtered_circles, -1, (0, 255, 0), 2)
        cv2.imshow("found_buoys", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def lemme_see_it(img, tag = 'yeet'):
    cv2.imshow(tag, img)
    cv2.waitKey(0) 

def rainbows(img):
    img = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)  
    img = cv2.applyColorMap(img, cv2.COLORMAP_VIRIDIS)
    return img
'''
from wand.image import Image
from wand.display import display

def the_polar_express(notpolar, display_results = False):
    notpolar = rainbows(notpolar)

    lemme_see_it(notpolar)

    img_rgb = cv2.cvtColor(notpolar, cv2.COLOR_BGR2RGB)

    img = Image.from_array(notpolar)
    img.resize(140, 92)
    img.virtual_pixel = 'black'
    img.distort('polar', (0,))
    #display(img)
    
    arr = np.array(img)
    scale_percent = 300 # percent of original size
    width = int(arr.shape[1] * scale_percent / 100)
    height = int(arr.shape[0] * scale_percent / 100)
    dim = (width, height)
  
    # resize image
    resized = cv2.resize(arr, dim, interpolation = cv2.INTER_AREA)
    lemme_see_it(resized)

    # img = notpolar.astype(np.float32)
    # value = np.sqrt(((img.shape[0]/2.0)**2.0)+((img.shape[1]/2.0)**2.0))
    
    # polar_image = cv2.linearPolar(img,(img.shape[0]/2, img.shape[1]/2), value, cv2.WARP_FILL_OUTLIERS)
    
    # polar_image = polar_image.astype(np.uint8)
    # lemme_see_it(polar_image)
'''

def mask_sonar_image(img, display_results=False):
    """ Get mask of potential objects in a sonar image

    Args:
        img (ndarray): Image to mask
        display_results (bool, optional): Whether to display the results. Defaults to False.

    Returns:
        ndarray: Mask image
    """
    lower_color_bounds = (40, 80, 0)  # Filter out lower values (ie blue)
    upper_color_bounds = (230, 250, 255)  # Filter out too high values
    mask = cv2.inRange(img, lower_color_bounds, upper_color_bounds)

    if display_results:
        cv2.imshow("mask", mask)
        cv2.waitKey(0)

    return mask


def test_img_proc(img, func=find_buoy):
    if isinstance(img, np.ndarray):
        sonar_img = img
    elif isinstance(img, str):
        if img.endswith('.bin'):
            sonar_img = build_sonar_img_from_log_file('SampleTylerData.bin')
        elif img.endswith('.npy'):
            sonar_img = np.load(img)

    locations = func(sonar_img, display_results=True)
    print(locations)


if __name__ == "__main__":
    # test_img_proc(os.path.join(os.path.dirname(__file__), 'sampleData', 'SampleTylerData.bin'), find_buoy)
    # test_img_proc(os.path.join(os.path.dirname(__file__), 'sampleData', 'buoy.npy'), find_buoy)
    test_img_proc(os.path.join(os.path.dirname(__file__), 'sampleData', 'gate.npy'), the_polar_express)

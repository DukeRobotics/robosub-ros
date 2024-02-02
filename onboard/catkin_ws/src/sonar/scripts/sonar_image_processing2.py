import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image

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


def createImage(img, start_angle=0, center_angle=90):
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
    angle = img.shape[0]
    radius = img.shape[1]

    center = start_angle + angle/2 # find center of the scan arc

    width = 2*radius + 1
    height = radius + 1

    polarImg = np.zeros((height, width, 3))

    x_ax = np.linspace(-radius, radius, width, dtype=np.int32)
    y_ax = np.linspace(0, radius, height, dtype=np.int32)
    xx, yy = np.meshgrid(x_ax, y_ax)

    theta = np.rad2deg(np.arctan2(yy, xx))*200/180
    r = np.sqrt(xx**2 + yy**2)

    np.round(theta)
    np.round(r)
    
    theta = theta.astype(np.int32)
    r = r.astype(np.int32)

    for x in x_ax:
        for y in y_ax:
            theta_pt = theta[y][x + radius] - int(center_angle*200/180 - center) # shift angles to center the to center_angle
            r_pt = r[y][x + radius] # x + radius to convert x values into index values

            if (theta_pt < angle and theta_pt > 0 and r_pt < radius):
                polarImg[radius - y][x + radius] = img[theta_pt][r_pt]/255 # radius - y flips to face the scan upward
            else:
                polarImg[radius - y][x + radius] = np.zeros(3)
    
    return polarImg


def addContours(image, lower_bound=(0, 0.5, 0), upper_bound=(1, 1, 1), kernel=(17, 17), area_threshold=5000):
    """ gaussian blurs the image then adds contours
    
    Args:
        image (ndarray): image to process
        lower_bound (tuple): lower bound when thresholding the image to find contour regions
        upper_bound (tuple): upper bound when thresholding the image to find contour regions
        kernel (tuple): kernel to use when applying gaussian blur
        area_threshold (int or float): only shows contours that are bigger than this value
    """
    mask = cv2.inRange(image, lower_bound, upper_bound)

    blurred_img = cv2.GaussianBlur(mask, kernel, 0)

    contours, hierarchy = cv2.findContours(blurred_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    shapes = []
    for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        if moments['m00'] > area_threshold:
            shapes.append(contours[i])
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(image, (cx, cy), 8, (255, 0, 0), -1)
            cv2.drawContours(image, contours, i, (255, 0, 0), 2)
    printCirularity("Circularity: ", shapes)
    return image

def printCirularity(contours):

    for con in contours:
        perimeter = cv2.arcLength(con, True)
        area = cv2.contourArea(con)
        if perimeter == 0:
            break
        circularity = 4*np.pi*(area/perimeter*perimeter)
        print(circularity)


def main():
    path = 'onboard/catkin_ws/src/sonar/scripts/sampleData/Sonar_Image.jpeg'
    # path = 'onboard/catkin_ws/src/sonar/scripts/sampleData/buoy.npy'
    
    image = addContours(createImage(getImage(path)))

    # plt.imshow(image)
    # plt.show()

    scale_factor = 0.5
    new_height = int(image.shape[0] * scale_factor)
    new_width = int(image.shape[1] * scale_factor)
    image = cv2.resize(image, (new_width, new_height))

    if path.endswith(".npy"):
        image = image[:, :, 0]
        image = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        image = cv2.applyColorMap(image, cv2.COLORMAP_VIRIDIS)

    cv2.imshow("polar image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main2():
    sonar_img = np.load('onboard/catkin_ws/src/sonar/sampleData/sonar_sweep_1.npy')
    sonar_img = sonar_img.astype(np.uint8) 

    sonar_img_col = cv2.cvtColor(sonar_img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    sonar_img_col = cv2.applyColorMap(sonar_img, cv2.COLORMAP_VIRIDIS)

    sonar_img_polar = createImage(sonar_img)
    resized_img = cv2.resize(sonar_img_polar, (sonar_img_polar.shape[1] // 2, sonar_img_polar.shape[0] // 2))
    print(sonar_img_polar.shape)

    # plt.imshow(sonar_img)
    # plt.show()
    cv2.imshow('sonar image', resized_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main2()
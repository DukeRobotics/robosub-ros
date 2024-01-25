import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib import transforms
from PIL import Image

def getImage(path):
    """ turn jpeg into numpy array
    
    Args:
        path (string): path of the image

    Returns:
        ndarray: image
    """
    img = Image.open(path)
    npImg = np.asarray(img)
    return npImg


def createImage(path):
    """ create a cartesian top down image from a polar image with
    x coordinates as the distance away from the sonar and
    y coordinates as angles (in gradians)
    
    Args:
        path (string): path of the image

    Returns:
        ndarray: image
    """
    npImg = getImage(path)

    radius = npImg.shape[1]
    polarImg = np.zeros((radius + 1, radius + 1, 3))

    x = np.linspace(0, radius, radius + 1)
    y = np.linspace(0, radius, radius + 1)
    xx, yy = np.meshgrid(x, y)

    theta = np.rad2deg(np.arctan2(yy, xx))*200/180    
    r = np.sqrt(xx**2 + yy**2)
    r = r/(np.sqrt(2))

    np.round(theta)
    np.round(r)
    
    theta = theta.astype(np.int32)
    r = r.astype(np.int32)

    for j in range(radius):
        for i in range(radius + 1):
            polarImg[radius-j][i] = npImg[theta[j][i]][r[j][i]]/255
    
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

    for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        if moments['m00'] > area_threshold:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(image, (cx, cy), 8, (255, 0, 0), -1)
            cv2.drawContours(image, contours, i, (255, 0, 0), 2)


def main():
    path1 = 'onboard/catkin_ws/src/sonar/scripts/sampleData/Sonar_Image.jpeg'
    path2 = 'onboard/catkin_ws/src/sonar/scripts/sampleData/Sonar_Image2.jpeg'
    path3 = 'onboard/catkin_ws/src/sonar/scripts/sampleData/Sonar_Image3.jpeg'

    paths = [path1, path2, path3]
    images = []

    for path in paths:
        image = createImage(path)
        addContours(image)
        images.append(image)

    fig, ax = plt.subplots(1, len(images))

    tr = transforms.Affine2D().rotate_deg(135)

    for i in range(len(images)):
        ax[i].imshow(images[i], transform=tr+ax[i].transData)
        
        ax[i].set_xlim(-1*images[i].shape[0]*np.sqrt(2), 0)
        ax[i].set_ylim(-1*images[i].shape[0]*np.sqrt(2)/2, images[i].shape[0]*np.sqrt(2)/2)

        ax[i].set_aspect('equal', 'box')
    
    plt.show()

if __name__ == '__main__':
    main()
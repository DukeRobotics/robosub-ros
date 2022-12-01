import cv2


def find_gate_posts(self, img, display_results=False):
    """ Find gate posts from a sonar scan image """

    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image,5) # blur image

    mask = mask_sonar_image(cm_image)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cm_circles = sorted(cm_circles, key=cv2.contourArea, reverse=True)
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 200), cm_circles)) 
    cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cm_circles)) 

    if(len(cm_circles) < 1):
        print("Not enough circles found")
        return None

    filtered_circles = cm_circles[0:2]

    circle_positions = []
    for circle in filtered_circles:  #find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX,cY))

    if display_results:
        cv2.imshow("image", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def find_bouy(self, img, display_results=False):
    """ Find buoys from a sonar scan image """

    greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

    cm_copy_image = cm_image
    cv2.copyTo(cm_image, cm_copy_image)
    cm_image = cv2.medianBlur(cm_image,5) # blur image

    mask = mask_sonar_image(cm_image)

    if display_results:
        cv2.imshow("image", mask)
        cv2.waitKey(0)

    cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 100), cm_circles)) 

    cm_circles = sorted(cm_circles, key=lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x))), reverse=True)

    filtered_circles = cm_circles[0:1]

    circle_positions = []
    for circle in filtered_circles:  #find center of circle code
        M = cv2.moments(circle)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        circle_positions.append((cX, cY))

    if display_results:
        cv2.drawContours(cm_copy_image, filtered_circles, -1, (0,255,0), 2)
        cv2.imshow("image", cm_copy_image)
        cv2.waitKey(0)

    return circle_positions


def mask_sonar_image(cm_image):
    """ Get mask of potential objects in a sonar image """
    lower_color_bounds = (40,80,0) # filter out lower values (ie blue)
    upper_color_bounds = (230,250,255) #filter out too high values
    mask = cv2.inRange(cm_image, lower_color_bounds, upper_color_bounds)
    return mask


def increase_brightness(img, value=20):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

import math
import cv2
import numpy as np

# use in acc method
# greyscale_image = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
# cm_image = cv2.applyColorMap(greyscale_image, cv2.COLORMAP_VIRIDIS)

cm_image = cv2.imread('onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\Sonar_Image3.jpeg', cv2.IMREAD_COLOR)

cm_copy_image = cm_image
cv2.copyTo(cm_image, cm_copy_image)
cm_image = cv2.medianBlur(cm_image,5) # blur image

lower_color_bounds = (40,80,0) # filter out lower values (ie blue)
upper_color_bounds = (230,250,255) #filter out too high values
mask = cv2.inRange(cm_image,lower_color_bounds,upper_color_bounds)

cv2.imshow("image", mask)
cv2.waitKey(0)




cm_circles = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
cm_circles = list(filter(lambda x: (cv2.contourArea(x) > 100), cm_circles)) 

cm_circles = sorted(cm_circles, key=lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x))), reverse=True)
#cm_circles = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cm_circles)) 



filtered_circles = cm_circles[0:1]

circle_positions = []
for circle in filtered_circles:  #find center of circle code
    M = cv2.moments(circle)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    circle_positions.append((cX,cY))
    #print("object at " + "x: " + str(cX) + "  Y: " + str(cY)  +  " has circularity : " + str(perimeter**2/ (4*math.pi*area) ) )
    #cv2.circle(cm_copy_image, (cX, cY), 3, (255, 255, 255), -1)

cv2.drawContours(cm_copy_image, filtered_circles, -1, (0,255,0), 2)
cv2.imshow("image", cm_copy_image)
cv2.waitKey(0)
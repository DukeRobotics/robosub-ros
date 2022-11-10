import math
import cv2
import numpy as np


img = cv2.imread("onboard\\catkin_ws\\src\\sonar\\scripts\\sampleData\\bruh.jpg", cv2.IMREAD_COLOR)

john = img
cv2.copyTo(img, john)

lower_color_bounds = (40,100,0)
upper_color_bounds = (200,250,255)
mask = cv2.inRange(img,lower_color_bounds,upper_color_bounds )

cv2.imshow("image", mask)
cv2.waitKey(0)
# mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

#th, threshed = cv2.threshold(mask, 100, 255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)

cnts = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
cnts = list(filter(lambda x: (cv2.contourArea(x) > 300), cnts)) 
cnts = list(filter(lambda x: (cv2.arcLength(x, True)**2/(4*math.pi*cv2.contourArea(x)) < 5.4), cnts)) 





coolcont = cnts[0:2]
cv2.drawContours(john,coolcont, -1, (0,255,0), 2)


for con in coolcont:
    perimeter = cv2.arcLength(con,True)

    area = cv2.contourArea(con)
  
    M = cv2.moments(con)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    print("object at " + "x: " + str(cX) + "  Y: " + str(cY)  +  " has circularity : " + str(perimeter**2/ (4*math.pi*area) ) )
    cv2.circle(john, (cX, cY), 3, (255, 255, 255), -1)

# detector = cv2.SimpleBlobDetector_create()
# keypoints = detector.detect(mask)
# newimg = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


#ret, thresh1 = cv2.threshold(img,120,180,cv2.THRESH_BINARY)


cv2.imshow("image", john)
cv2.waitKey(0)
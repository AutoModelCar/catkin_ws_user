#!/usr/bin/env python2
import sys
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def nothing(x):
    pass

img = cv2.imread("/home/tinieblas/catkin_ws/odo_lights_I.png")
cv2.imshow('original', img)
cv2.namedWindow('image')
cv2.createTrackbar('h', 'image', 0, 255, nothing)
cv2.createTrackbar('s', 'image', 0, 255, nothing)
cv2.createTrackbar('v', 'image', 0, 255, nothing)
cv2.createTrackbar('H', 'image', 0, 255, nothing)
cv2.createTrackbar('S', 'image', 0, 255, nothing)
cv2.createTrackbar('V', 'image', 0, 255, nothing)

while (1):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)



    #get current positions of four trackbars
    h = cv2.getTrackbarPos('h', 'image')
    s = cv2.getTrackbarPos('s', 'image')
    v = cv2.getTrackbarPos('v', 'image')
    H = cv2.getTrackbarPos('H', 'image')
    S = cv2.getTrackbarPos('S', 'image')
    V = cv2.getTrackbarPos('V', 'image')
    #
    lowLimit = np.array((h, s, v)) # (0, 91, 42) #
    highLimit = np.array((H, S, V)) # (184, 248, 239)
    mask = cv2.inRange(hsv, lowLimit, highLimit)

    cv2.imshow('image', mask)
    cv2.waitKey(1)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()





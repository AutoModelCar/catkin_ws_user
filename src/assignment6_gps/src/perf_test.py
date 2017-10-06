import cv2

from balloon_detector import BalloonDetector
import numpy as np

d = BalloonDetector()
img = cv2.imread('img.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hue = hsv[:, :, 0].astype(np.float)

pos_avg = 0
for i in xrange(1000):
    pos_avg += d.calculate_position(hsv, hue)

print('done: %s' % pos_avg)



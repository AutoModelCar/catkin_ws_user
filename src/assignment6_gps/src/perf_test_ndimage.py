import cv2

from balloon_detector_ndimage import BalloonDetector
import numpy as np

d = BalloonDetector(close_iter=0)
img = cv2.imread('img.png')


pos_avg = 0
for i in xrange(1000):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.float)
    hue_radians = np.pi / 90 * hue

    pos_avg += d.calculate_position(hsv, hue_radians, 0)

print('done: %s' % pos_avg)



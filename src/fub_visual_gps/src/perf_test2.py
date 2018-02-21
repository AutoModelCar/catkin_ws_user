import timeit

setup = 'import cv2; import numpy as np; hsv = cv2.cvtColor(cv2.imread("img.png"), cv2.COLOR_BGR2HSV)'

timeit.main(['-s', setup, 'hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]'])
timeit.main(['-s', setup, 'hue, saturation, value = map(np.squeeze, np.dsplit(hsv, 3))'])
timeit.main(['-s', setup, 'hue, saturation, value = map(np.squeeze, np.split(hsv, 3, axis=2))'])
timeit.main(['-s', setup, 'hue, saturation, value = cv2.split(hsv)'])

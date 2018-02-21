# Standard imports
import cv2
import numpy as np

# Read image
im = cv2.imread("image-rect-100.png")

# Set up the detector with default parameters.
params = cv2.SimpleBlobDetector_Params()

params.filterByCircularity = True
params.minCircularity = 0.01

params.filterByArea = True
#params.minArea = 50


params.minThreshold = 10
print(params)
print(params.minThreshold)


detector = cv2.SimpleBlobDetector_create(params)

# Detect blobs.
keypoints = detector.detect(im)

print(keypoints)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

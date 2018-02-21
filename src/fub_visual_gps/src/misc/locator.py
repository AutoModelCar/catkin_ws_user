#!/usr/bin/env python2
import numpy as np

from scipy import stats

import cv2

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

np.set_printoptions(precision=2, suppress=True)


l_width = 1
l_len = 8
l_color = (250, 0, 200)
font = cv2.FONT_HERSHEY_SIMPLEX
def draw_cross(img, cluster, label):
    pos = np.round(cluster[:2]).astype(np.int)
    off_x, off_y = [l_len, 0], [0, l_len]
    cv2.line(img, tuple(pos - off_x), tuple(pos + off_x), l_color, l_width)
    cv2.line(img, tuple(pos - off_y), tuple(pos + off_y), l_color, l_width)
    cv2.putText(img, label, tuple(pos + [l_len + 4, 4]), font, 0.35, (0, 0, 0))
    cv2.putText(img, label, tuple(pos + [l_len + 3, 3]), font, 0.35, (255, 200, 255))


class RectImageHandler:
    def __init__(self):
        self.image_pub_marked = rospy.Publisher("/assignment6/image_marked", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # convert to gray single channel by taking the "Y" channel of the yuv representation
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)[:, :, 0]

        # convert to black and white (binary image)
        _, thresh = cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)

        # publish black and white image
        self.image_pub_thresh.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))

        # get white pixel coordinates
        ys, xs = np.where(thresh != 0)
        points = np.column_stack((xs, ys))
        params_a = find_best_params(points)  # find first line

        # only look at outliers for finding second line
        outliers = points[np.where(np.logical_not(get_inliers(points, *params_a)))]

        # find a second line (only in the outliers of the first)
        params_b = find_best_params(outliers)

        # create a color version of the black and white picture so we can draw on it
        bgr_thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        colors = ((0, 0, 250), (250, 0, 0))
        l_width = 2
        max_x = thresh.shape[1]
        for (slope, intercept), color in zip((params_a, params_b), colors):
            cv2.line(bgr_thresh, (0, int(intercept)), (max_x, int(slope * max_x + intercept)), color, l_width)

        self.image_pub_marked.publish(self.bridge.cv2_to_imgmsg(bgr_thresh, 'bgr8'))


def main():
    rospy.init_node('assignment5_segmentizer')
    RectImageHandler()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()

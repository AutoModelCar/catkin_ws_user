#!/usr/bin/env python2

import rospy
from balloon_detector import BalloonDetector, balloons
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np


class ImageHandler:
    def __init__(self):
        self.image_pub_marked = rospy.Publisher("/assignment6/image_marked_rect", Image, queue_size=1, latch=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_undistorted", Image, self.callback, queue_size=10)
        self.detector = BalloonDetector()

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        self.detector.detect_balloons(img)
        print(self.detector.balloon_positions)

        self.image_pub_marked.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        # TODO: Calculate position


def main():
    np.set_printoptions(precision=2, suppress=True)  # less verbose numpy printing
    rospy.init_node('assignment6_rect_localisation')
    ImageHandler()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()

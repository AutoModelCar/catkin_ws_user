#!/usr/bin/env python2

import rospy
from balloon_detector import BalloonDetector
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

import math


class ImageHandler:
    def __init__(self, img_pub=False):
        if img_pub:
            self.image_pub_marked = rospy.Publisher("/assignment6/image_marked_ang", Image, queue_size=200, latch=True)
        else:
            self.image_pub_marked = None
        self.odom_pub = rospy.Publisher("/assignment6/odom", Odometry, queue_size=200)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        self.detector = BalloonDetector()

        pose_covar = PoseWithCovariance(Pose(Point(0, 0, 0), Quaternion()), None)
        self.odom = Odometry(Header(frame_id='visualgps'), 'base_link', pose_covar, None)

    def callback(self, data):
        t_start = rospy.Time.now()
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        xy = self.detector.calculate_best_position(img)

        if self.image_pub_marked is not None:
            self.detector.draw_markers(img)
            self.image_pub_marked.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        # Don't publish a pose if location can't be reliably determined
        if xy is None:
            print("No location found")
            return

        angle = self.detector.calculate_angle()

        header = self.odom.header
        header.seq = data.header.seq
        header.stamp = data.header.stamp

        pose = self.odom.pose.pose
        pos = pose.position
        pos.x, pos.y = xy

        quaternion = pose.orientation
        quaternion.z, quaternion.w = math.sin(angle / 2), math.cos(angle / 2)

        self.odom_pub.publish(self.odom)

        print('%-30s angle: %6.1f img_age: %5.3f calc: %5.3f' %
              (xy, np.rad2deg(angle), to_secs(t_start - data.header.stamp), to_secs(rospy.Time.now() - t_start)))


def to_secs(duration):
    """
    :type duration: rospy.Duration
    """
    return duration.secs + 1e-9 * duration.nsecs


def main():
    rospy.init_node('assignment6_angle_localisation')
    ImageHandler()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()

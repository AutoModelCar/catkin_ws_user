#!/usr/bin/env python2
from __future__ import print_function

from collections import Counter

import rospy
from nav_msgs.msg import Odometry


class FpsCounter:
    def __init__(self):
        self.sub = rospy.Subscriber("/assignment6/odom", Odometry, self.callback, queue_size=100)
        self.counter = Counter()
        self.offset = None

    def callback(self, data):
        current = data.header.stamp.secs

        if self.offset is None:
            current, self.offset = 0, current
        else:
            current -= self.offset

        self.counter[current] += 1

        for sec, count in sorted(self.counter.items()):
            if sec < current:
                print('sec %4d: %3d FPS' % (sec, count))
                del self.counter[sec]


def main():
    rospy.init_node('fps_counter')
    FpsCounter()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()

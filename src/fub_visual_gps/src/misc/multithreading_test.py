#!/usr/bin/env python2
import time
import rospy
from std_msgs.msg import Int16


class ThreadTester:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/testa", Int16, self.test_a, queue_size=2)
        self.gps_sub = rospy.Subscriber("/testb", Int16, self.test_b, queue_size=2)
        self.last_a_msg = None

    def test_a(self, data):
        print('test_a: %s' % data)
        self.last_a_msg = data

    def test_b(self, data):
        print('> test_b: %s' % data)
        t = self.last_a_msg
        while t is self.last_a_msg:
            time.sleep(0.05)
        print('got a msg!')
        print('< test_b: %s' % data)


def main():
    rospy.init_node('multithreading_test')
    ThreadTester()  # constructor creates publishers / subscribers
    rospy.spin()


if __name__ == '__main__':
    main()

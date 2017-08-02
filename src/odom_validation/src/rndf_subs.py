#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
from matplotlib import animation
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

x = np.array([])
y = np.array([])
z = np.array([]) #sequence number
file2write=open("rndf_base.txt",'w')

def point_callback(data):
    file2write.write(str(data.point.x))
    file2write.write(",")
    file2write.write(str(data.point.y))
    file2write.write("\n")


def main(args):
    rospy.init_node('rndf_subs', anonymous = False)
    image_sub = rospy.Subscriber("/clicked_point",PointStamped, point_callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

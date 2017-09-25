#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
#from heading_msg.msg import Yaw

file2write=open("odom_car.txt",'w')

def odom_callback(data):
    global x,y,z
    file2write.write(str(data.header.stamp.secs))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.x))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.y))
    file2write.write("\n")


def yaw_callback(data):
    print(data)

def main(args):
    rospy.init_node('odom_plotter', anonymous = False)
    print('Publish odometry from car/rosbag to record to odom_car.txt ')
    image_sub = rospy.Subscriber("/odom",Odometry, odom_callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

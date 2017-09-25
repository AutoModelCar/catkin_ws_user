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
from geometry_msgs.msg import Twist
from heading_msg.msg import Yaw
import math
Yaw_pub = rospy.Publisher("/Yaw2",Yaw, queue_size =1)
count = 0
def yaw_callback(data):
    global count
    yaw_msg = Yaw()
    yaw_msg.header.seq = count
    yaw_msg.header.stamp = rospy.get_rostime()
    yaw_msg.angle = data.data
    Yaw_pub.publish(yaw_msg)
    #print("publi")
    count = count+1

def main(args):
    global x,y
    rospy.init_node('data_plotter', anonymous = False)
    image_sub0 = rospy.Subscriber("/model_car/yaw",Float32, yaw_callback)
    #image_sub1 = rospy.Subscriber("/motor_control/twist",Twist, speed_callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

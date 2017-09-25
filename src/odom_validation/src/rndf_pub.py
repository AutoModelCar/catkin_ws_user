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
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import time

x = np.array([])
y = np.array([])
lane =open("/home/korivi/Desktop/frei_traj/code/RNDF_Creation/ll_sparse.txt",'r').read()

rlane =open("/home/korivi/Desktop/frei_traj/code/RNDF_Creation/rl_sparse.txt",'r').read()

def main(args):
    rospy.init_node('rndf_pubs', anonymous = False)
    goal_pub = rospy.Publisher("/goal_markers", MarkerArray, queue_size = 1)
    dataArray = lane.split('\n')
    newMarkerArray = MarkerArray()
    i = 0
    color = [1.0, 0.0 ,0.0]
    for eachLine in dataArray:
        if len(eachLine)>1:
            x1,y1 = eachLine.split(',')
            newMarkerArray.markers.append(Marker())
            newMarkerArray.markers[i].header.frame_id = "map"
            newMarkerArray.markers[i].id = i;
            newMarkerArray.markers[i].type = 2
            newMarkerArray.markers[i].scale.x = 0.01
            newMarkerArray.markers[i].scale.z = 0.01
            newMarkerArray.markers[i].scale.y = 0.03
            newMarkerArray.markers[i].color.a = 1.0
            newMarkerArray.markers[i].color.r = color[0]
            newMarkerArray.markers[i].color.g = color[1]
            newMarkerArray.markers[i].color.b = color[2]
            newMarkerArray.markers[i].pose.position.x = float(x1)
            newMarkerArray.markers[i].pose.position.y = float(y1)
            i = i+1
            #print('marker added')
    dataArray = rlane.split('\n')
    color = [0.0, 1.0 ,0.0]
    for eachLine in dataArray:
        if len(eachLine)>1:
            x1,y1 = eachLine.split(',')
            newMarkerArray.markers.append(Marker())
            newMarkerArray.markers[i].header.frame_id = "map"
            newMarkerArray.markers[i].id = i;
            newMarkerArray.markers[i].type = 2
            newMarkerArray.markers[i].scale.x = 0.01
            newMarkerArray.markers[i].scale.z = 0.01
            newMarkerArray.markers[i].scale.y = 0.03
            newMarkerArray.markers[i].color.a = 1.0
            newMarkerArray.markers[i].color.r = color[0]
            newMarkerArray.markers[i].color.g = color[1]
            newMarkerArray.markers[i].color.b = color[2]
            newMarkerArray.markers[i].pose.position.x = float(x1)
            newMarkerArray.markers[i].pose.position.y = float(y1)
            i = i+1

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        goal_pub.publish(newMarkerArray)
        print('Published')
        rate.sleep()
    #rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

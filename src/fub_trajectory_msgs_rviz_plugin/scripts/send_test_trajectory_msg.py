#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped
import math
import rospy

topic = 'test_trajectory'
publisher = rospy.Publisher(topic, Trajectory)

rospy.init_node('send_path')

delta = 0
while not rospy.is_shutdown():

        t = Trajectory()
        t.header.frame_id = "/base_link"
        t.header.stamp = rospy.Time.now()
        
        num_points = 100
        time_step = 0.1
        for i in range( 0, num_points ):
                point = TrajectoryPoint()
                point.pose.position.x = 10.0 * i / num_points - 5
                point.pose.position.y = math.sin( 10.0 * i / num_points + delta )
                point.pose.position.z = 0
                point.pose.orientation.x = 0
                point.pose.orientation.y = 0
                point.pose.orientation.z = 0
                point.pose.orientation.w = 1

                point.time_from_start = rospy.rostime.Duration(i*time_step)
                point.velocity.linear.x = abs(math.sin( 10.0 * i / num_points + delta ))*15.0
                point.velocity.angular.z = abs(math.sin( 10.0 * i / num_points + delta ))

                t.trajectory.append( point )

        publisher.publish( t )
        
        delta += .1
        
        rospy.sleep(0.03)

#!/usr/bin/env python2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose, Point, Quaternion, PoseWithCovariance
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, Float32
from std_msgs.msg import Header
import math
from numpy.linalg import inv



class kalman_filter:
    def __init__(self):
       
        pose_covar = PoseWithCovariance(Pose(Point(0, 0, 0), Quaternion()), None)
        self.odom = Odometry(Header(frame_id='map'), 'base_link_2', pose_covar, None)
        self.local_yaw=0
        self.global_yaw=-10
        self.initial_yaw=-10

        self.kalman_h=np.zeros(3).transpose()
        self.kalman_y=np.zeros(3).transpose()
        self.kalman_x=np.zeros(3).transpose()
        self.kalman_v=10**(-6)*np.eye(3)
        self.kalman_p=10**(-9)*np.eye(3)
        self.kalman_w=10**(-2)*np.eye(3)
        self.kalman_d_h=np.eye(3)
        self.kalman_d_y=np.eye(3)

        self.init=0
        self.v=0
        self.vdt=0

        self.last_time = rospy.get_rostime()
        self.kalman_odom_pub = rospy.Publisher("/kalman/odom", Odometry, queue_size=2)
        self.global_odom_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.callbackGlobalPos, queue_size=1)
        # self.twist_sub = rospy.Subscriber("model_car/twist", Twist, self.callbacktwist, queue_size=1)
        # self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callbackHead, queue_size=1)
        self.local_odom_sub = rospy.Subscriber("/odom", Odometry, self.callbackLocalPos, queue_size=1)        

    def kalman_update(self):
        Error=self.kalman_y-self.kalman_h
        # the covariance of the observation noise;
        kalman_v=10**(-6)*np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        kalman_s=(self.kalman_d_y*self.kalman_p)*self.kalman_d_y.transpose()+kalman_v
        kalman_k=0.1*np.matrix([[1,0,0],[0,1,0],[0,0,0.1]])
        self.kalman_x=self.kalman_h+kalman_k*Error
        self.kalman_p=self.kalman_p-kalman_k*self.kalman_d_h*self.kalman_p
        while (self.kalman_x[2]>3.14):
            self.kalman_x[2]=self.kalman_x[2]-6.28
        while (self.kalman_x[2]<-3.14):
            self.kalman_x[2]=self.kalman_x[2]+6.28
        self.publish_kalman_odom(self.kalman_x)

    def publish_kalman_odom(self, data):
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x=data.item(0)
        self.odom.pose.pose.position.y=data.item(1)
        angle = data.item(2)
        self.odom.pose.pose.orientation.z = math.sin(angle / 2) 
        self.odom.pose.pose.orientation.w = math.cos(angle / 2)
        self.kalman_odom_pub.publish(self.odom)

    def callbackLocalPos(self, data):
        v=data.twist.twist.linear.x
        current_time = rospy.get_rostime();
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.local_yaw) = euler_from_quaternion (orientation_list)
        if (self.init==0 and self.global_yaw!=-10):
            self.init=1
        # kalamFilter() prediction
        if (self.init==1):
            self.v=v
            dt=(current_time - self.last_time).nsecs*(10**(-9))
            self.vdt=self.v*dt
            self.kalman_h=np.matrix([np.cos(self.local_yaw)*self.vdt+self.kalman_x.item(0),np.sin(self.local_yaw)*self.vdt+self.kalman_x.item(1),self.local_yaw]).transpose()
            self.kalman_d_h=np.matrix([[1,0,-np.sin(self.local_yaw)*self.vdt],[0,1,np.cos(self.local_yaw)*self.vdt],[0,0,1]])
            #the covariance of the process noise
            self.kalman_w=10**(-2)*np.matrix([[1,0,0],[0,1,0],[0,0,6]])
            self.kalman_p=self.kalman_d_h*self.kalman_p*self.kalman_d_h.transpose()+self.kalman_w
            self.kalman_x=self.kalman_h
            self.publish_kalman_odom(self.kalman_h)
        self.last_time = current_time

    # def callbacktwist(self, data):
    #     v=round(data.linear.x / (5.5))*(0.031)
    #     current_time = rospy.get_rostime();
    #     # kalamFilter() prediction
    #     if (self.init==1):
    #         self.v=v
    #         dt=(current_time - self.last_time).nsecs*(10**(-9))
    #         self.vdt=self.v*dt
    #         self.kalman_h=np.matrix([np.cos(self.local_yaw)*self.vdt+self.kalman_x.item(0),np.sin(self.local_yaw)*self.vdt+self.kalman_x.item(1),self.local_yaw]).transpose()
    #         self.kalman_d_h=np.matrix([[1,0,-np.sin(self.local_yaw)*self.vdt],[0,1,np.cos(self.local_yaw)*self.vdt],[0,0,1]])
    #         self.kalman_v=10**(-2)*np.matrix([[1,0,0],[0,1,0],[0,0,6]])
    #         self.kalman_p=self.kalman_d_h*self.kalman_p*self.kalman_d_h.transpose()+self.kalman_v
    #         publish_kalman_odom(self.kalman_h)
    #     self.last_time = current_time

    # def callbackHead(self, data):
    #     if (self.init==0 and self.global_yaw!=-10):
    #         self.init=1
    #         self.initial_yaw=data.data* (3.14/180.0)-self.global_yaw
    #         self.local_yaw=self.global_yaw
    #     else:
    #         if (self.initial_yaw!=-10):
    #             yaw=data.data* (3.14/180.0)-self.initial_yaw
    #             while (yaw>3.14):
    #                 yaw=yaw-6.28;
    #             while (yaw<-3.14):
    #                 yaw=yaw+6.28;
    #             self.local_yaw=yaw

    def callbackGlobalPos(self, data):

        global_x = data.pose.pose.position.x
        global_y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.global_yaw) = euler_from_quaternion (orientation_list)
        if (self.init==0):
            self.kalman_x=np.matrix([global_x,global_y,self.global_yaw]).transpose()
            self.kalman_y=self.kalman_x
        else:
            self.kalman_y=np.matrix([global_x,global_y,self.global_yaw]).transpose()
            self.kalman_d_y=np.eye(3)
            #kalamFilter() update
            self.kalman_update()
def main():
    rospy.init_node('kalman_filter')
    kalman_filter()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
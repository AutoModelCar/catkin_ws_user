#!/usr/bin/env python

# --- imports ---
import rospy
from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# --- definitions ---
epsilon = 0.05   # allowed inaccuracy for distance calculation
speed_rpm = 200
angle_left = 30
angle_straight = 90
angle_right = 150
last_odom = None
is_active = False


def callbackOdom(msg):
    global last_odom
    last_odom = msg


def waitForFirstOdom():
    while not rospy.is_shutdown() and last_odom is None:
        rospy.loginfo(
            "%s: No initial odometry message received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(1.0)


def callbackForward(msg):
    drive(msg.data, "callbackForward", speed_rpm, angle_straight)


def callbackBackward(msg):
    drive(msg.data, "callbackBackward", -speed_rpm, angle_straight)


def callbackForwardLeft(msg):
    drive(msg.data, "callbackForwardLeft", speed_rpm, angle_left)


def callbackForwardRight(msg):
    drive(msg.data, "callbackForwardRight", speed_rpm, angle_right)


def callbackBackwardLeft(msg):
    drive(msg.data, "callbackBackwardLeft", -speed_rpm, angle_left)


def callbackBackwardRight(msg):
    drive(msg.data, "callbackBackwardRight", -speed_rpm, angle_right)


def drive(distance, command, speed, angle):
    global is_active

    rospy.loginfo("%s: Running %s(%f)", rospy.get_caller_id(), command, distance)
    if distance <= 0:
        rospy.logerr(
            "%s: Error, distance argument has to be > 0! %f given",
            rospy.get_caller_id(),
            distance)
        return

    pub_info.publish("BUSY")
    if is_active:
        rospy.logwarn(
            "%s: Warning, another command is still active! Please wait and try again.",
            rospy.get_caller_id())
        return

    is_active = True

    # stop the car and set desired steering angle + speed
    pub_speed.publish(0)
    pub_stop_start.publish(1)
    rospy.sleep(1)
    pub_steering.publish(angle)
    pub_stop_start.publish(0)
    rospy.sleep(1)
    pub_speed.publish(speed)

    start_pos = last_odom.pose.pose.position
    current_distance = 0

    while not rospy.is_shutdown() and current_distance < (distance - epsilon):
        current_pos = last_odom.pose.pose.position
        current_distance = sqrt(
            (current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
        # rospy.loginfo("current distance = %f", current_distance)
        rospy.sleep(0.1)

    pub_speed.publish(0)
    is_active = False
    current_pos = last_odom.pose.pose.position
    current_distance = sqrt((current_pos.x - start_pos.x)
                            ** 2 + (current_pos.y - start_pos.y)**2)
    pub_info.publish("FINISHED")

    rospy.loginfo(
        "%s: Finished %s(%f)\nActual travelled distance = %f",
        rospy.get_caller_id(),
        command,
        distance,
        current_distance)


# --- main program ---


# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our node so that multiple instances can
# run simultaneously.
rospy.init_node("simple_drive_control")

# read parameters from parameter server (launch file or rosparam set), if
# existing
if rospy.has_param("speed_rpm"):
    speed_rpm = rospy.get_param("speed_rpm")

# create subscribers and publishers
sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=100)
# wait for first odometry message, till adverting subscription of commands
waitForFirstOdom()
sub_forward = rospy.Subscriber(
    "simple_drive_control/forward", Float32, callbackForward, queue_size=10)
sub_backward = rospy.Subscriber(
    "simple_drive_control/backward", Float32, callbackBackward, queue_size=10)
sub_forward_left = rospy.Subscriber(
    "simple_drive_control/forward_left", Float32, callbackForwardLeft, queue_size=10)
sub_forward_right = rospy.Subscriber(
    "simple_drive_control/forward_right", Float32, callbackForwardRight, queue_size=10)
sub_backward_left = rospy.Subscriber(
    "simple_drive_control/backward_left", Float32, callbackBackwardLeft, queue_size=10)
sub_backward_right = rospy.Subscriber(
    "simple_drive_control/backward_right", Float32, callbackBackwardRight, queue_size=10)

pub_stop_start = rospy.Publisher(
    "manual_control/stop_start",
    Int16,
    queue_size=100)
pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
pub_steering = rospy.Publisher(
    "steering",
    UInt8,
    queue_size=100)
pub_info = rospy.Publisher("simple_drive_control/info", String, queue_size=100)

rospy.loginfo(rospy.get_caller_id() + ": started!")

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

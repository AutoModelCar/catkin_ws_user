#!/usr/bin/env python

# --- imports ---
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

# --- definitions ---
epsilon = 0.1   # allowed inaccuracy for distance calculation
speed_rpm = 50
last_odom = None
is_active = False


def callbackOdom(msg):
    global last_odom
    last_odom = msg


def waitForFirstOdom():
    while not rospy.is_shutdown() and last_odom is None:
        rospy.loginfo(
            "%s: No initial odometry message received. Waiting for message...",
            str(rospy.get_name))
        rospy.sleep(1.0)


def callbackForward(msg):
    drive(msg.data, "callbackForward", speed_rpm, 0)


def callbackBackward(msg):
    drive(msg.data, "callbackBackward", -speed_rpm, 0)


def callbackForwardLeft(msg):
    drive(msg.data, "callbackForwardLeft", speed_rpm, -20)


def callbackForwardRight(msg):
    drive(msg.data, "callbackForwardRight", speed_rpm, 20)


def callbackBackwardLeft(msg):
    drive(msg.data, "callbackBackwardLeft", -speed_rpm, -20)


def callbackBackwardRight(msg):
    drive(msg.data, "callbackBackwardRight", -speed_rpm, 20)


def drive(distance, command, speed, angle):
    global is_active

    rospy.loginfo("%s: Running %s(%d)", rospy.get_name, command, distance)
    if distance <= 0:
        rospy.logerror(
            "%s: Error, distance argument has to be > 0! %d given",
            rospy.get_name,
            distance)
        return

    if is_active:
        rospy.logwarn(
            "%s: Warning, another command is still active! Please wait and try again.",
            rospy.get_name)
        return

    is_active = True

    # stop the car and set desired steering angle + speed
    pub_speed.publish(0)
    rospy.sleep(1)
    pub_steering.publish(angle)
    rospy.sleep(1)
    pub_speed.publish(speed)

    start_pose = last_odom.pose.pose
    current_distance = 0

    while not rospy.is_shutdown() and current_distance < (distance - epsilon):
        current_pose = last_odom.pose.pose
        current_distance = sqrt(
            (current_pose.x - start_pose.x)**2 + (current_pose.y - start_pose.y)**2)
        rospy.sleep(0.1)

    pub_speed.publish(0)
    is_active = False
    current_pose = last_odom.pose.pose
    current_distance = sqrt((current_pose.x - start_pose.x)
                            ** 2 + (current_pose.y - start_pose.y)**2)
    rospy.loginfo(
        "%s: Finished %s(%d)\nActual travelled distance = %d",
        rospy.get_name,
        command,
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
    "simple_drive_control/forward", Int32, callbackForward, queue_size=10)
sub_backward = rospy.Subscriber(
    "simple_drive_control/backward", Int32, callbackBackward, queue_size=10)
sub_forward_left = rospy.Subscriber(
    "simple_drive_control/forward_left", Int32, callbackForwardLeft, queue_size=10)
sub_forward_right = rospy.Subscriber(
    "simple_drive_control/forward_right", Int32, callbackForwardRight, queue_size=10)
sub_backward_left = rospy.Subscriber(
    "simple_drive_control/backward_left", Int32, callbackBackwardLeft, queue_size=10)
sub_backward_right = rospy.Subscriber(
    "simple_drive_control/backward_right", Int32, callbackBackwardRight, queue_size=10)

pub_speed = rospy.Publisher("manual_control/speed", Int32, queue_size=100)
pub_steering = rospy.Publisher(
    "manual_control/steering",
    Int32,
    queue_size=100)

rospy.loginfo(rospy.get_caller_id() + ": started!")

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

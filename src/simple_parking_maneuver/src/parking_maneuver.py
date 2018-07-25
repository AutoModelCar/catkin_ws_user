#!/usr/bin/env python

# --- imports ---
import rospy
from simple_parking_maneuver.srv import *
from std_msgs.msg import Float32
from std_msgs.msg import String

# --- definitions ---
last_driving_control_info = ""


def callbackDrivingControl(msg):
    last_driving_control_info = msg.data

def callbackBackwardLongitudinal(request):
    rospy.loginfo(rospy.get_caller_id() + ": callbackBackwardLongitudinal, direction = " + request.direction)

    if request.direction == "left":
        driving_direction_pub1 = pub_back_left
        driving_direction_pub2 = pub_back_right
    elif request.direction == "right":
        driving_direction_pub1 = pub_back_right
        driving_direction_pub2 = pub_back_left
    else:
        return ParkingManeuverResponse(
            "ERROR: Request can only be 'left' or 'right'")

    driving_direction_pub1.publish(0.30)

#     rospy.sleep(10)
#     pub_back.publish(0.10)

    rospy.sleep(10)
    driving_direction_pub2.publish(0.30)

    rospy.sleep(10)
    pub_forward.publish(0.1)

    rospy.sleep(10)
    return ParkingManeuverResponse("FINISHED")


# --- main program ---

# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our node so that multiple instances can
# run simultaneously.
rospy.init_node("simple_parking_maneuver")

# create subscribers and publishers
sub_info = rospy.Subscriber(
    "simple_drive_control/info", String, callbackDrivingControl, queue_size=10)
sub_backward_longitudinal = rospy.Service(
    "simple_parking_maneuver/backward_longitudinal",
    ParkingManeuver,
    callbackBackwardLongitudinal)

pub_back_left = rospy.Publisher(
    "simple_drive_control/backward_left",
    Float32,
    queue_size=10)
pub_back_right = rospy.Publisher(
    "simple_drive_control/backward_right",
    Float32,
    queue_size=10)
pub_back = rospy.Publisher(
    "simple_drive_control/backward",
    Float32,
    queue_size=10)
pub_forward = rospy.Publisher(
    "simple_drive_control/forward",
    Float32,
    queue_size=10)


rospy.loginfo(rospy.get_caller_id() + ": started!")

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()


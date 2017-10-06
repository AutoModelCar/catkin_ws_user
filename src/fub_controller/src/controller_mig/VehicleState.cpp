#include "VehicleState.h"

#include <tf/tf.h>


namespace fub {
namespace controller {
namespace mig {

void VehicleState::setup(ros::NodeHandle & nh)
{
    // todo: increase odom queue to at least 32
    mSubscriberOdom         = nh.subscribe("/odom", 1, &VehicleState::odometryCallback, this, ros::TransportHints().tcpNoDelay());
    mSubscriberPlannedPath  = nh.subscribe("planned_path", 1, &VehicleState::plannedPathCallback, this, ros::TransportHints().tcpNoDelay());
}

void VehicleState::plannedPathCallback(const fub_trajectory_msgs::TrajectoryConstPtr & msg)
{
    mPath = *msg;
}
void VehicleState::odometryCallback(const nav_msgs::OdometryConstPtr & msg)
{
    mEgoStatePose = msg->pose;
    mCurrentSpeedFrontAxleCenter =(double) msg->twist.twist.linear.x;
    tf::pointMsgToTF(mEgoStatePose.pose.position, mVehiclePosition);
    mLastOdomTimeStampReceived = msg->header.stamp;
}

double VehicleState::getVehicleYaw() const
{
    return tf::getYaw(mEgoStatePose.pose.orientation);// * radians;
}

}
}
}

#pragma once


#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <fub_trajectory_msgs/Trajectory.h>

#include <ros/ros.h>
#include <tf/tf.h>

namespace fub {
namespace controller {
namespace mig {

/** Data holding class of current vehicle state. Subscribes to the required
 ** topics to be always up-to-date.
 **
 ** TODO: instead of subscribing individually to the topics, consider using
 **       approximate_time. This would simplify the issue that values are changed
 **       concurrently, as now use of a critical section is easier (cheaper).
 */
class VehicleState
{
public:
    void setup(ros::NodeHandle & nh);

    /** Retrieve the vehicle's yaw orientation in the world
     **
     ** @return the vehicle's yaw in the world
     */
    double getVehicleYaw() const;


protected:
    void odometryCallback(const nav_msgs::OdometryConstPtr & msg);
    void plannedPathCallback(const fub_trajectory_msgs::TrajectoryConstPtr & msg);

public:
    /// last received pose (from odometry message)
    geometry_msgs::PoseWithCovariance mEgoStatePose;

    /// last received position (from odometry message)
    tf::Point mVehiclePosition;

    /// timestamp of last received odometry message
    ros::Time mLastOdomTimeStampReceived;

    /// the currently planned path
    fub_trajectory_msgs::Trajectory mPath;
    double mCurrentSpeedFrontAxleCenter;


private:
    ros::Subscriber mSubscriberOdom;
    ros::Subscriber mSubscriberPlannedPath;
};

}
}
}

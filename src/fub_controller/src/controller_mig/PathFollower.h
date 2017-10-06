#pragma once

#include <fub_controller/ControllerMigPathConfig.h>
#include "util/DynamicConfigurationHelper.h"
#include "util/Spline.h"
#include "VehicleState.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <ros/publisher.h>

namespace fub {
namespace controller {
namespace mig {

class PathFollower
    : public util::DynamicConfigurationHelper<fub_controller::ControllerMigPathConfig>
{
public:
    /**
     **
     ** @param frame
     ** @param nodeHandle
     ** @param privateNodeHandle
     */
    void setup(std::string const & frame, ros::NodeHandle & nodeHandle, ros::NodeHandle & privateNodeHandle);

    void updateSplineAndClosestPoints(VehicleState const & currentVehicleState);
    double getBearingAngle(VehicleState const & currentVehicleState);
    double getWantedVelocity(VehicleState const & currentVehicleState);

protected:

    void calcSteeringLookaheadPoint(VehicleState const & currentVehicleState);

    int getClosestIdxOnPlan(tf::Point const & referencePoint, fub_trajectory_msgs::Trajectory const & plan);
    
    void publishSteeringLookAheadPointMarker() const;
    void publishVelocityLookAheadPointMarker() const;
    void publishClosestPointInSpaceMarker() const;
    void publishClosestPointInTimeMarker() const;

    util::Spline mSpline;
    double mClosestParamInSpace;
    double mClosestParamInTime;

    tf::Point mClosestPointInSpace;
    tf::Point mClosestPointInTime;

    tf::Point mSteeringLookaheadPoint;
    tf::Point mVelocityLookaheadPoint;

    std::string mFrameID;

    ros::Publisher mClosestPointInSpacePublisher;
    ros::Publisher mClosestPointInTimePublisher;
    ros::Publisher mSteeringLookAheadPointPublisher;
    ros::Publisher mVelocityLookAheadPointPublisher;
    ros::Publisher mSplineSamplePublisher;
    ros::Publisher mSplineDerivativeSamplePublisher;
};


}
}
}

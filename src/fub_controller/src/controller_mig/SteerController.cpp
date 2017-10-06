#include "SteerController.h"

#include "util/Math.h"

#include <boost/algorithm/clamp.hpp>

namespace fub {
namespace controller {
namespace mig {

using namespace util;


void SteerController::setup(ros::NodeHandle & nodeHandle, ros::NodeHandle & privateNodeHandle)
{
    setupConfiguration("Steering", privateNodeHandle);
}

double SteerController::control(const VehicleState &vehicleState, double deltaT, double wantedSteerAngle, double currentSteerAngle)
{
    // Note re the value 0.04: This used to be the period of the controller (25 Hz)
    // a long time ago, and had been kept to get the same controller results.

    double newSteerError = (wantedSteerAngle - currentSteerAngle);
        ROS_INFO("newSteerError%f",newSteerError);

    Math::normalizeAngle(newSteerError);
        ROS_INFO("normalizeAngle%f",newSteerError);

    mSteerIntegral += newSteerError * deltaT / (0.04);
    mSteerIntegral = boost::algorithm::clamp(mSteerIntegral, -mConfig.limit_steer_integral, mConfig.limit_steer_integral);

    double currentSpeed = vehicleState.mCurrentSpeedFrontAxleCenter;
    double steerOutput =
          newSteerError * Math::boundedLinearInterpolation(currentSpeed, mConfig.low_speed_value, mConfig.high_speed_value, mConfig.kp_steer_low_speed_value, mConfig.kp_steer_high_speed_value)
        + mSteerIntegral * Math::boundedLinearInterpolation(currentSpeed, mConfig.low_speed_value, mConfig.high_speed_value, mConfig.ki_steer_low_speed_value, mConfig.ki_steer_high_speed_value)
        + (newSteerError - mOldSteerError) * (0.04) / deltaT * Math::boundedLinearInterpolation(currentSpeed, mConfig.low_speed_value, mConfig.high_speed_value, mConfig.kd_steer_low_speed_value, mConfig.kd_steer_high_speed_value);

    mOldSteerError = newSteerError;
    ROS_INFO("steerOutput%f",steerOutput);
    return boost::algorithm::clamp(steerOutput, -1.0, 1.0);
}


}
}
}

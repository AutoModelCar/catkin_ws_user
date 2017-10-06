#include "ControllerMig.h"

#include <ros/ros.h>

#include <limits>
#include <string.h>

// in controlSteering only
#include <boost/algorithm/clamp.hpp>
#include <tf/tf.h>
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

namespace fub {
namespace controller {
namespace mig {

using namespace fub::controller::util;

void ControllerMig::onInit()
{
    mSteeringAngleNormalized=0.0;
    // advertise publishers (before subscribing to the vehicle state / setting up timer)
    mWantedNormSteerAnglePublisher = getNodeHandle().advertise<std_msgs::Float32>("command/normalized_steering_angle", 10);
    mWantedSpeedPublisher          = getNodeHandle().advertise<std_msgs::Int16>("manual_control/speed", 10);
    mWantedSteeringAnglePublisher  = getNodeHandle().advertise<std_msgs::Int16>("manual_control/steering", 10);


    // set up dynamic reconfigure service (before any configuration is used) - this will start the timer
    setupConfiguration("Main", getPrivateNodeHandle());



    // initialize the sub-components (before subscribers)
    mPathFollower.setup(getPrivateNodeHandle().param<std::string>("odom_frame", "odom"), getNodeHandle(), getPrivateNodeHandle());
    mSteerPIDController.setup(getNodeHandle(), getPrivateNodeHandle());

    // register the subscribers that receive the vehicle state information
    mVehicleState.setup(getNodeHandle());
}


// default parameter set
void ControllerMig::configurationCallback(fub_controller::ControllerMigConfig & config, uint32_t level) {
    DynamicConfigurationHelper::configurationCallback(config, level);

    // create / update up the timer
    ros::Duration timerPeriod = ros::Duration(1.0 / mConfig.execution_frequency);
    if (not mTimer.isValid()) {
        mTimer = getNodeHandle().createTimer(timerPeriod, &ControllerMig::callbackTimer, this);
    } else {
        mTimer.setPeriod(timerPeriod);
    }
}


void ControllerMig::callbackTimer(const ros::TimerEvent &)
{
    // create a copy of the vehicle state - we do NOT want these values to
    // change while we are working with them
    // TODO: ensure that data does not change during copying
    VehicleState currentVehicleState = mVehicleState;

    update(currentVehicleState);
}


void ControllerMig::update(VehicleState const & currentVehicleState)
{
    // if we have no path, don't do anything
    if (currentVehicleState.mPath.trajectory.empty()) {
        ROS_DEBUG_THROTTLE(1, "Controller has no path");

        //publish last/default messages
        //TODO unset activation? brake?
        publish(currentVehicleState);

        return;
    }

    // TODO: should we assume a fixed frequency, or rather track the delta
    //       time ourselves?
    double deltaT = 1.0 / (mConfig.execution_frequency);

    //update spline for path following
    mPathFollower.updateSplineAndClosestPoints(currentVehicleState);
    
    // get bearing angle
    double const angleBearing = mPathFollower.getBearingAngle(currentVehicleState);

    //get wanted velocity
    double const wantedSpeed = mPathFollower.getWantedVelocity(currentVehicleState);


    // steer correction
    double angleHead = currentVehicleState.getVehicleYaw();
    if (not std::isnan(angleHead) and not std::isnan(angleBearing)) { // TODO proper safety checks
        mSteeringAngleNormalized = mSteerPIDController.control(currentVehicleState, deltaT, angleBearing, angleHead);
    } else {
        // TODO: should we disengage?
        ROS_ERROR("angleHead or angleBearing is nan, driving straight");
        ROS_ERROR("angleBearing: %f angleHead: %f current speed: %f", angleBearing, angleHead, currentVehicleState.mCurrentSpeedFrontAxleCenter);
        mSteeringAngleNormalized = 0;
    }
        ROS_INFO("steerOutput%f",mSteeringAngleNormalized);

    // TODO: this makes no sense: shouldn't this be mGear or better yet, the
    //       current gear?
/*
    if (mDriveGear == fub_mig_can_msgs::MIGGearStatus::GEAR_POSITION_R) { //Drive Backwards
        // when driving backwards, we need to increase our steering angle to keep
        // the rear axle center point following the plan - the factor of 3.0 was
        // determined experimentally
        steerAngle = steerAngle * 3.0;
    }
*/

    publish(currentVehicleState);

    publishWantedSpeedAndFrontWheelAngle(wantedSpeed, mSteeringAngleNormalized);
}

void ControllerMig::publish(VehicleState const & vehicleState)
{
    ros::Time now = ros::Time::now();

    std_msgs::Float32 steerMsg;
    steerMsg.data = mSteeringAngleNormalized;
    mWantedNormSteerAnglePublisher.publish(steerMsg);
}

void ControllerMig::publishWantedSpeedAndFrontWheelAngle(double speed, double wheelAngle)
{
    // publish wanted speed
        std_msgs::Int16 wantedSpeedMsg;
        wantedSpeedMsg.data        = static_cast<int16_t>(speed *(-1489.36));
        mWantedSpeedPublisher.publish(wantedSpeedMsg);

    // publish wanted steering angle
        std_msgs::Int16 wantedAngleMsg;
        wantedAngleMsg.data = static_cast<int16_t>(90.0 * wheelAngle) + 90;
        mWantedSteeringAnglePublisher.publish(wantedAngleMsg);
}

}
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fub::controller::mig::ControllerMig, nodelet::Nodelet);

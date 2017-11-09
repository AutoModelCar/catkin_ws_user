#include "PathFollower.h"

#include "util/Math.h"

#include <boost/algorithm/clamp.hpp>

#include <visualization_msgs/Marker.h>

#include <nav_msgs/Path.h>

namespace fub {
namespace controller {
namespace mig {

using namespace util;

void PathFollower::setup(std::string const & frame, ros::NodeHandle & nodeHandle, ros::NodeHandle & privateNodeHandle)
{
    mFrameID = frame;

    setupConfiguration("PathFollowing", privateNodeHandle);

    mVelocityLookAheadPointPublisher    = privateNodeHandle.advertise<visualization_msgs::Marker>("velocity_lookahead_point_marker", 0);
    mSteeringLookAheadPointPublisher    = privateNodeHandle.advertise<visualization_msgs::Marker>("steering_lookahead_point_marker", 0);
    mClosestPointInSpacePublisher = privateNodeHandle.advertise<visualization_msgs::Marker>("closest_point_in_space_marker", 0);
    mClosestPointInTimePublisher = privateNodeHandle.advertise<visualization_msgs::Marker>("closest_point_in_time_marker", 0);
    mSplineSamplePublisher      = privateNodeHandle.advertise<nav_msgs::Path>("sampled_spline_debug", 10);
    mSplineDerivativeSamplePublisher      = privateNodeHandle.advertise<nav_msgs::Path>("sampled_spline_derivative_debug", 10);
}

void PathFollower::updateSplineAndClosestPoints(VehicleState const & currentVehicleState)
{
    //update spline
    mSpline.createSpline(currentVehicleState.mPath);
    mSpline.publishSampledSpline(mSplineSamplePublisher, mFrameID);
    mSpline.publishSampledSplineDerivative(mSplineDerivativeSamplePublisher, mFrameID);
    
    //update closest point in space
    mClosestParamInSpace = mSpline.getClosestParam(currentVehicleState.mVehiclePosition);

    mClosestPointInSpace.setX(mSpline.getX(mClosestParamInSpace));
    mClosestPointInSpace.setY(mSpline.getY(mClosestParamInSpace));
    mClosestPointInSpace.setZ(mSpline.getZ(mClosestParamInSpace));

    publishClosestPointInSpaceMarker();

    //update closest point in time
    ros::Duration durationFromStartOfPlan = currentVehicleState.mLastOdomTimeStampReceived - currentVehicleState.mPath.header.stamp;
    mClosestParamInTime = fmax(mSpline.getFirstParam(), fmin(durationFromStartOfPlan.toSec(), mSpline.getLastParam()));

    mClosestPointInTime.setX(mSpline.getX(mClosestParamInTime));
    mClosestPointInTime.setY(mSpline.getY(mClosestParamInTime));
    mClosestPointInTime.setZ(mSpline.getZ(mClosestParamInTime));
    
    publishClosestPointInTimeMarker();
}


double PathFollower::getBearingAngle(VehicleState const & currentVehicleState)
{
    calcSteeringLookaheadPoint(currentVehicleState);
    publishSteeringLookAheadPointMarker();
    
    //calc bearing
    tf::Vector3 diffPosTrajCarVec = mSteeringLookaheadPoint - currentVehicleState.mVehiclePosition;
    double angleBearing = atan2(diffPosTrajCarVec[1], diffPosTrajCarVec[0]);

    return angleBearing;
}

void PathFollower::calcSteeringLookaheadPoint(VehicleState const & currentVehicleState)
{
    double lookaheadTime = mConfig.steering_lookahead_time;

    double param = fmax(mSpline.getFirstParam(), fmin(mClosestParamInSpace + lookaheadTime, mSpline.getLastParam()));

    mSteeringLookaheadPoint.setX(mSpline.getX(param));
    mSteeringLookaheadPoint.setY(mSpline.getY(param));
    mSteeringLookaheadPoint.setZ(mSpline.getZ(param));

    double lookaheadDist = mClosestPointInSpace.distance(mSteeringLookaheadPoint);
    
    //we want to look ahead at least min_lookahead_dist in space (if the plan is long enough)
    while (lookaheadDist < mConfig.steering_min_lookahead_dist and (mClosestParamInSpace + lookaheadTime) <= mSpline.getLastParam()) {
        lookaheadTime += 0.0001;  //TODO make this a binary search
        param = fmax(mSpline.getFirstParam(), fmin(mClosestParamInSpace + lookaheadTime, mSpline.getLastParam()));
    
        mSteeringLookaheadPoint.setX(mSpline.getX(param));
        mSteeringLookaheadPoint.setY(mSpline.getY(param));
        mSteeringLookaheadPoint.setZ(mSpline.getZ(param));
        
        lookaheadDist = mClosestPointInSpace.distance(mSteeringLookaheadPoint);
    }
    
    //check if lookahead point is still too close
    if (lookaheadDist < mConfig.steering_min_lookahead_dist) {
        double distMissing = mConfig.steering_min_lookahead_dist - lookaheadDist;

        double carOrientVecX = cos(currentVehicleState.getVehicleYaw());
        double carOrientVecY = sin(currentVehicleState.getVehicleYaw());

        double carPosX = currentVehicleState.mVehiclePosition[0];
        double carPosY = currentVehicleState.mVehiclePosition[1];
        double closestPosX = mClosestPointInSpace[0];
        double closestPosY = mClosestPointInSpace[1];
        
        double fromCarToClosestX = closestPosX - carPosX;
        double fromCarToClosestY = closestPosY - carPosY;
        
        double dotProduct = (fromCarToClosestX*carOrientVecX)+(fromCarToClosestY*carOrientVecY);
        
        if (dotProduct < 0) { //we are at the end of the plan
            double distCarToLookahead = sqrt((carPosX-closestPosX)*(carPosX-closestPosX) + (carPosY-closestPosY)*(carPosY-closestPosY));
            distMissing += distCarToLookahead;
        }
        
        //use orientation of last point of plan to project a point the min dist
        tf::Quaternion lastOrientationQuat;
        tf::quaternionMsgToTF(currentVehicleState.mPath.trajectory.back().pose.orientation, lastOrientationQuat);
        tf::Vector3 orientationVec = tf::Vector3(1,0,0).rotate(lastOrientationQuat.getAxis(),lastOrientationQuat.getAngle());
        mSteeringLookaheadPoint = mSteeringLookaheadPoint + orientationVec.normalized()* distMissing;
        
    }
}

double PathFollower::getWantedVelocity(VehicleState const & currentVehicleState)
{
    if (mConfig.use_static_wanted_speed) {
        return mConfig.static_wanted_speed;// * meters_per_second;
    }

    //calc velocity lookahead param/point
    double lookaheadTime = mConfig.velocity_lookahead_time;
    double lookaheadParam = fmax(mSpline.getFirstParam(), fmin(mClosestParamInTime+lookaheadTime, mSpline.getLastParam()));

    //get velocity at lookahead param
    tf::Vector3 velocityVectorAtLookahead(mSpline.getVelX(lookaheadParam),mSpline.getVelY(lookaheadParam),mSpline.getVelZ(lookaheadParam));
    double wantedVelocity = velocityVectorAtLookahead.length();//*meters/seconds;
    
    //get lookahead point
    mVelocityLookaheadPoint.setX(mSpline.getX(lookaheadParam));
    mVelocityLookaheadPoint.setY(mSpline.getY(lookaheadParam));
    mVelocityLookaheadPoint.setZ(mSpline.getZ(lookaheadParam));

    publishVelocityLookAheadPointMarker();
    
    //adjust velocity due to error between current position in space and planned position in time
    double distToPlannedPoint = mClosestPointInSpace.distance(mVelocityLookaheadPoint);
    double factor = mConfig.velocity_lookahead_dist_error_factor;
    
    double maxCorrection = mConfig.velocity_max_offset;
    double velocityCorrection = fmin(distToPlannedPoint*factor, maxCorrection);//meters/seconds
    
    if (lookaheadParam>mClosestParamInSpace) {
        wantedVelocity += velocityCorrection;
    } else {
        wantedVelocity -= velocityCorrection;
    }
    
    //reduce velocity if we are to far away from the spline
    double x = mClosestPointInSpace.x()-currentVehicleState.mVehiclePosition.x();
    double y = mClosestPointInSpace.y()-currentVehicleState.mVehiclePosition.y();
    double distCarToTrajectory2d = sqrt(x*x+y*y);
    //TODO make this params?
    double safetyVel = 0.3;//*meters/seconds;
    double safeDist = 0.2; //within this distance from the spline it is ok to drive the wanted velocity
    double safeDist2 = 2.0; //above this distance from the spline we want to drive with safety velocity
    
    double safeWantedVelocity = safetyVel;
    if (distCarToTrajectory2d < safeDist) {
        //ok, we are safe
        safeWantedVelocity = wantedVelocity;
    } else if (distCarToTrajectory2d < safeDist2){
        //interpolate
        safeWantedVelocity = ((((safeDist2-distCarToTrajectory2d) - safeDist2)/(safeDist-safeDist2))*(wantedVelocity-safetyVel)+safetyVel);//*meters/seconds;
    }
    return safeWantedVelocity;
}

int PathFollower::getClosestIdxOnPlan(tf::Point const & referencePoint, fub_trajectory_msgs::Trajectory const & plan)
{
    double currentClosestDistance = DBL_MAX;
    int indexOfCurrentBestPoint = 0;

    //stupid linear search:
    for (int i=0; i < plan.trajectory.size(); ++i) {
        tf::Point pointOnPlan;
        tf::pointMsgToTF(plan.trajectory[i].pose.position, pointOnPlan);
        double dist = pointOnPlan.distance(referencePoint);
        if (dist<currentClosestDistance) {
            currentClosestDistance = dist;
            indexOfCurrentBestPoint = i;
        }
    }
    
    return indexOfCurrentBestPoint;
}


void PathFollower::publishSteeringLookAheadPointMarker() const
{
    //TODO make this a marker array
    if (mSteeringLookAheadPointPublisher.getNumSubscribers() == 0) {
        return;
    }

    //publish ros marker for the steering lookahead point
    visualization_msgs::Marker marker;
    marker.header.frame_id = mFrameID;
    marker.header.stamp = ros::Time();
    marker.ns = "Look-Ahead Point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mSteeringLookaheadPoint.getX();
    marker.pose.position.y = mSteeringLookaheadPoint.getY();
    marker.pose.position.z = mSteeringLookaheadPoint.getZ();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    mSteeringLookAheadPointPublisher.publish(marker);
}

void PathFollower::publishVelocityLookAheadPointMarker() const
{
    //TODO make this a marker array
    if (mVelocityLookAheadPointPublisher.getNumSubscribers() == 0) {
        return;
    }

    //publish ros marker for the steering lookahead point
    visualization_msgs::Marker marker;
    marker.header.frame_id = mFrameID;
    marker.header.stamp = ros::Time();
    marker.ns = "Look-Ahead Point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mVelocityLookaheadPoint.getX();
    marker.pose.position.y = mVelocityLookaheadPoint.getY();
    marker.pose.position.z = mVelocityLookaheadPoint.getZ();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    mVelocityLookAheadPointPublisher.publish(marker);
}

void PathFollower::publishClosestPointInSpaceMarker() const
{
    if (mSpline.getLastParam() == -1) { // TODO: add "check" function
        //spline not initialized
        return;
    }

    if (mClosestPointInSpacePublisher.getNumSubscribers() == 0) {
        return;
    }

    //publish ros marker for the closest point
    visualization_msgs::Marker marker_debug;
    marker_debug.header.frame_id = mFrameID;
    marker_debug.header.stamp = ros::Time();
    marker_debug.ns = "Closest Point";
    marker_debug.id = 0;
    marker_debug.type = visualization_msgs::Marker::SPHERE;
    marker_debug.action = visualization_msgs::Marker::ADD;
    marker_debug.pose.position.x = mClosestPointInSpace.getX();
    marker_debug.pose.position.y = mClosestPointInSpace.getY();
    marker_debug.pose.position.z = mClosestPointInSpace.getZ();


    marker_debug.pose.orientation.x = 0.0;
    marker_debug.pose.orientation.y = 0.0;
    marker_debug.pose.orientation.z = 0.0;
    marker_debug.pose.orientation.w = 1.0;
    marker_debug.scale.x = 0.1;
    marker_debug.scale.y = 0.1;
    marker_debug.scale.z = 0.1;
    marker_debug.color.a = 1.0; // Don't forget to set the alpha!
    marker_debug.color.r = 0.0;
    marker_debug.color.g = 0.0;
    marker_debug.color.b = 1.0;

    mClosestPointInSpacePublisher.publish(marker_debug);
}

void PathFollower::publishClosestPointInTimeMarker() const
{
    if (mSpline.getLastParam() == -1) { // TODO: add "check" function
        //spline not initialized
        return;
    }

    if (mClosestPointInTimePublisher.getNumSubscribers() == 0) {
        return;
    }

    //publish ros marker for the closest point
    visualization_msgs::Marker marker_debug;
    marker_debug.header.frame_id = mFrameID;
    marker_debug.header.stamp = ros::Time();
    marker_debug.ns = "Closest Point";
    marker_debug.id = 0;
    marker_debug.type = visualization_msgs::Marker::SPHERE;
    marker_debug.action = visualization_msgs::Marker::ADD;
    marker_debug.pose.position.x = mClosestPointInTime.getX();
    marker_debug.pose.position.y = mClosestPointInTime.getY();
    marker_debug.pose.position.z = mClosestPointInTime.getZ();


    marker_debug.pose.orientation.x = 0.0;
    marker_debug.pose.orientation.y = 0.0;
    marker_debug.pose.orientation.z = 0.0;
    marker_debug.pose.orientation.w = 1.0;
    marker_debug.scale.x = 0.1;
    marker_debug.scale.y = 0.1;
    marker_debug.scale.z = 0.1;
    marker_debug.color.a = 1.0; // Don't forget to set the alpha!
    marker_debug.color.r = 0.0;
    marker_debug.color.g = 0.0;
    marker_debug.color.b = 1.0;

    mClosestPointInTimePublisher.publish(marker_debug);
}

}
}
}

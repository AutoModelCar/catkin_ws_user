#include "Spline.h"

#include <nav_msgs/Path.h>

namespace fub {
namespace controller {
namespace util {

void Spline::createSpline(fub_trajectory_msgs::Trajectory const & plan)
{
    // read path x and ys into separate arrays
    int numberOfPointsIntoSpline = plan.trajectory.size();
    ecl::Array<double> x_set(numberOfPointsIntoSpline);
    ecl::Array<double> y_set_x(numberOfPointsIntoSpline);
    ecl::Array<double> y_set_y(numberOfPointsIntoSpline);
    ecl::Array<double> y_set_z(numberOfPointsIntoSpline);

    for (int i = 0 ; i < numberOfPointsIntoSpline; i++) {
        x_set[i] = plan.trajectory.at(i).time_from_start.toSec();
        y_set_x[i] = plan.trajectory.at(i).pose.position.x;
        y_set_y[i] = plan.trajectory.at(i).pose.position.y;
        y_set_z[i] = plan.trajectory.at(i).pose.position.z;
    }

    //calculate velocity vector with regard to global coordinates
    tf::Vector3 frontVelocity;
    tf::vector3MsgToTF(plan.trajectory.front().velocity.linear, frontVelocity);
    tf::Quaternion frontOrientation;
    tf::quaternionMsgToTF(plan.trajectory.front().pose.orientation, frontOrientation);
    frontVelocity = frontVelocity.rotate(frontOrientation.getAxis(), frontOrientation.getAngle());
    
    tf::Vector3 backVelocity;
    tf::vector3MsgToTF(plan.trajectory.back().velocity.linear, backVelocity);
    tf::Quaternion backOrientation;
    tf::quaternionMsgToTF(plan.trajectory.back().pose.orientation, backOrientation);
    backVelocity = backVelocity.rotate(backOrientation.getAxis(), backOrientation.getAngle());
    
    //create splines for each dimension from the arrays
    mSpline_x = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_x, frontVelocity.getX(), backVelocity.getX());
    mSpline_y = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_y, frontVelocity.getY(), backVelocity.getY());
    mSpline_z = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set_z, frontVelocity.getZ(), backVelocity.getZ());
    
    mFirstParam = plan.trajectory.front().time_from_start.toSec();
    mLastParam = plan.trajectory.back().time_from_start.toSec();
}


double Spline::getClosestParam(tf::Point imuPos)
{
    double stepSize = 4.0;
    double discountFactor = 0.5;
    double minParam = mFirstParam;
    double maxParam = mLastParam;
    double closestParam = -1;
    double closestDistance = 200000000.0;
    double carPosX = imuPos.getX();
    double carPosY = imuPos.getY();
    double carPosZ = imuPos.getZ();

    double tempDistance = 0.0;

    while (stepSize > 0.0001) {
        closestDistance = 100000000.0;
        closestParam = -1;
        for (double i = minParam; i <= maxParam; i += stepSize) {
            tempDistance = pow((carPosX - mSpline_x(i)), 2) + pow((carPosY - mSpline_y(i)), 2) + pow((carPosZ - mSpline_z(i)), 2);

            if (closestDistance > tempDistance) {
                closestDistance = tempDistance;
                closestParam = i;
            }
        }
        minParam = fmax(minParam, closestParam - stepSize);
        maxParam = fmin(maxParam, closestParam + stepSize);
        stepSize *= discountFactor;
    }
    return closestParam;
}



void Spline::publishSampledSpline(ros::Publisher & publisher, std::string const & frame) const
{
    if (publisher.getNumSubscribers() == 0) {
        return;
    }

    nav_msgs::Path mSampledSplineDebug;
    mSampledSplineDebug.header.stamp = ros::Time::now();
    mSampledSplineDebug.header.frame_id = frame;

    mSampledSplineDebug.poses.clear();

    float xSampleDistance = 0.05; //in m

    for (float i = mFirstParam; i < mLastParam; i = i + xSampleDistance) {
        geometry_msgs::PoseStamped examplePose;

        examplePose.pose.position.x = mSpline_x(i);
        examplePose.pose.position.y = mSpline_y(i);
        examplePose.pose.position.z = mSpline_z(i);
        examplePose.pose.orientation.x = 0.0f;
        examplePose.pose.orientation.y = 0.0f;
        examplePose.pose.orientation.z = 0.0f;

        //push PoseStamped into Path
        mSampledSplineDebug.poses.push_back(examplePose);
    }

    publisher.publish(mSampledSplineDebug);
}

void Spline::publishSampledSplineDerivative(ros::Publisher & publisher, std::string const & frame) const
{
    if (publisher.getNumSubscribers() == 0) {
        return;
    }

    nav_msgs::Path mSampledSplineDebug;
    mSampledSplineDebug.header.stamp = ros::Time::now();
    mSampledSplineDebug.header.frame_id = frame;

    mSampledSplineDebug.poses.clear();

    float xSampleDistance = 0.05; //in m

    for (float i = mFirstParam; i < mLastParam; i = i + xSampleDistance) {
        geometry_msgs::PoseStamped examplePose;

        examplePose.pose.position.x = 0;
        examplePose.pose.position.y = i;
        examplePose.pose.position.z = tf::Vector3(mSpline_x.derivative(i),mSpline_y.derivative(i),mSpline_z.derivative(i)).length();
        examplePose.pose.orientation.x = 0.0f;
        examplePose.pose.orientation.y = 0.0f;
        examplePose.pose.orientation.z = 0.0f;

        //push PoseStamped into Path
        mSampledSplineDebug.poses.push_back(examplePose);
    }

    publisher.publish(mSampledSplineDebug);
}



}
}
}

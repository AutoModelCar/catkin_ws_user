#pragma once

#include <fub_trajectory_msgs/Trajectory.h>
#include <tf/tf.h>

#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>

namespace fub {
namespace controller {
namespace util {

class Spline {
public:
    /** Create spline from all points on path
     **
     ** @param plan
     */
    void createSpline(fub_trajectory_msgs::Trajectory const & plan);

    /**
     **
     ** @param imuPos
     ** @return
     */
    double getClosestParam(tf::Point imuPos);

    /**
     */
    double getFirstParam() const
    {
        return mFirstParam;
    }

    /**
     */
    double getLastParam() const
    {
        return mLastParam;
    }

    double getX(double x) const { return mSpline_x(x); }

    double getY(double y) const { return mSpline_y(y); }

    double getZ(double z) const { return mSpline_z(z); }

    double getVelX(double x) const { return mSpline_x.derivative(x); }

    double getVelY(double y) const { return mSpline_y.derivative(y); }

    double getVelZ(double z) const { return mSpline_z.derivative(z); }

    /**
     **
     ** @param publisher
     ** @param frame
     */
    void publishSampledSpline(ros::Publisher & publisher, std::string const & frame) const;

    /**
     **
     ** @param publisher
     ** @param frame
     */
    void publishSampledSplineDerivative(ros::Publisher & publisher, std::string const & frame) const;

protected:
    ecl::CubicSpline mSpline_x;
    ecl::CubicSpline mSpline_y;
    ecl::CubicSpline mSpline_z;

    double mFirstParam   = -1.0;
    double mLastParam    = -1.0;
};

}
}
}

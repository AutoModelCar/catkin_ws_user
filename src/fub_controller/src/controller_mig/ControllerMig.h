#include <nodelet/nodelet.h>
#include <ros/ros.h>

// configuration
#include <fub_controller/ControllerMigConfig.h>
#include "util/DynamicConfigurationHelper.h"

#include "PathFollower.h"
#include "SteerController.h"
#include "VehicleState.h"

#include <limits>
#include <string.h>

namespace fub {
namespace controller {
namespace mig {


/** Controller for MIG.
 **
 ** @ingroup @@
 */
class ControllerMig
    : public nodelet::Nodelet
    , public util::DynamicConfigurationHelper<fub_controller::ControllerMigConfig>
{

public:
    ControllerMig() = default;

    virtual ~ControllerMig() = default;

    /** Nodelet initialization
     */
    virtual void onInit() override;

protected:
    /** Callback for updated configuration parameters via dynamic_reconfigure.
     ** Overloads DynamicConfigurationHelper callback in order to update some
     ** member values.
     **
     ** @param config  The configuration structure with the updated values
     ** @param level   The OR-ed value of all parameter level values
     */
    void configurationCallback(fub_controller::ControllerMigConfig & config, uint32_t level) override;

    /** The callback for the timer that triggers the update.
     */
    void callbackTimer(const ros::TimerEvent&);


    /** This function checks for IMU orientation error while comparing motion vector with orientation vector
     ** idea - given three gps poses, the middle pose orientation should roughly equal the difference vector between first and last gps pose, assuming the pose is on rear axis
     **
     ** @param currentVehicleState  The current vehicle state.
     **
     ** @return the result of IMU orientation error
     **
     */
    void checkForIMUOrientation(VehicleState const & currentVehicleState);


    /** Update function, must be called at a fixed frequency.
     **
     ** @param currentVehicleState  The current vehicle state.
     **
     ** @return the result of the controller
     **
     ** @pre caller guarantees that currentVehicleState and its members are
     **      not changed while update() is running
     */
    void update(VehicleState const & currentVehicleState);

    /** Publish the results / requests.
     */
    void publish(VehicleState const & vehicleState);

    /** Publish wanted speed and front wheel angle
     */
    void publishWantedSpeedAndFrontWheelAngle(double speed, double wheelAngle);


protected:

    // publishers
    ros::Publisher mWantedNormSteerAnglePublisher;
    ros::Publisher mWantedSpeedPublisher;
    ros::Publisher mWantedSteeringAnglePublisher;

    // timer triggering our execution // TODO: use WallTimer?
    ros::Timer mTimer;

    /// the current vehicle state
    VehicleState mVehicleState;

    /* --- the controller output -------------------------------------------- */


    /// the steering wheel angle, normalized to [-1, 1]
    double mSteeringAngleNormalized;

    // the path follower
    PathFollower mPathFollower;

    // the controllers
    SteerController mSteerPIDController;

};

}
}
}

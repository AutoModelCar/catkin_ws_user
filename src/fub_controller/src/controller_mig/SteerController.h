#pragma once

#include "VehicleState.h"

#include <fub_controller/ControllerMigSteerConfig.h>
#include "util/DynamicConfigurationHelper.h"

namespace fub {
namespace controller {
namespace mig {

class SteerController
    : util::DynamicConfigurationHelper<fub_controller::ControllerMigSteerConfig>
{
public:
    /**
     **
     ** @param nodeHandle
     ** @param privateNodeHandle
     */
    void setup(ros::NodeHandle & nodeHandle, ros::NodeHandle & privateNodeHandle);

    /** Apply the PID controller.
     **
     ** @param vehicleState
     ** @param deltaT
     ** @param wantedSteerAngle   Wanted heading in world
     ** @param currentSteerAngle  Current heading in world
     **
     ** @return the steering wheel position (normalized to [-1., +1.])
     */
    double control(VehicleState const & vehicleState, double deltaT, double wantedSteerAngle, double currentSteerAngle);

protected:
    double mSteerIntegral = 0.0;
    double mOldSteerError = 0.0;
};


}
}
}

#include <gtest/gtest.h>
#include <controller_mig/VehicleState.h>

using namespace fub::controller::mig;

/* ------------------------------------------------------------------------- */

/** @ingroup @@_test
 */

class TestSteeringAndWheelRatio : public ::testing::Test
{
public:
    // initialization before each test case
    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }
};

// that that the conversion FROM steering wheel angle TO wheel angle has
// the correct order (i.e. not accidentially the other way round) by
// checking on it being less than 1 (the steering wheel always has higher
// angles than the front wheel)
TEST_F(TestSteeringAndWheelRatio, CorrectConversion)
{
    EXPECT_LT(VehicleState::steeringWheelAngleToWheelAngle(1), 1.);
    EXPECT_GT(VehicleState::wheelAngleToSteeringWheelAngle(1), 1.);
}

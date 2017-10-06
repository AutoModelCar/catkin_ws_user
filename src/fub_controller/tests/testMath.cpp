#include <gtest/gtest.h>

#include "util/Math.h"

#include <math.h>


using namespace fub::controller::util;

/* ------------------------------------------------------------------------- */

/** @ingroup @@_test
 */

class TestMath : public ::testing::Test
{
public:
    // initialization before each test case
    virtual void SetUp() override
    {
    }

    virtual void TearDown() override
    {
    }

protected:
    double getNormalizedAngleForTest(double angle)
    {
        Math::normalizeAngle(angle);
        return angle;
    }
};


/* ------------------------------------------------------------------------- */

TEST_F(TestMath, TestNormalizeAngle)
{
    EXPECT_FLOAT_EQ(0, getNormalizedAngleForTest(0));
    EXPECT_FLOAT_EQ(0.1, getNormalizedAngleForTest(0.1));
    EXPECT_FLOAT_EQ(M_PI, getNormalizedAngleForTest(M_PI));
    EXPECT_FLOAT_EQ(M_PI, getNormalizedAngleForTest(-M_PI));
    EXPECT_FLOAT_EQ(0.1*M_PI, getNormalizedAngleForTest(2.1*M_PI));
    EXPECT_FLOAT_EQ(0., getNormalizedAngleForTest(-2*M_PI));
    EXPECT_FLOAT_EQ(-0.1*M_PI, getNormalizedAngleForTest(-2.1*M_PI));
}


/* ------------------------------------------------------------------------- */

TEST_F(TestMath, TestBoundedLinearInterpolation)
{
    // points (0, 0) and (10, 10), i.e. f(x) = clamp(x, 0, 10)
    EXPECT_FLOAT_EQ(0, Math::boundedLinearInterpolation(0, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(0, Math::boundedLinearInterpolation(-10, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(1, Math::boundedLinearInterpolation(1, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(7, Math::boundedLinearInterpolation(7, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(10, Math::boundedLinearInterpolation(10, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(10, Math::boundedLinearInterpolation(10.1, 0, 10, 0, 10));
    EXPECT_FLOAT_EQ(10, Math::boundedLinearInterpolation(11, 0, 10, 0, 10));

    // points (-10, 10) and (10, -10), i.e. f(x) = -clamp(x, -10, 10)
    EXPECT_FLOAT_EQ(10, Math::boundedLinearInterpolation(-10, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(10, Math::boundedLinearInterpolation(-11, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(9, Math::boundedLinearInterpolation(-9, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(0, Math::boundedLinearInterpolation(0, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(-1, Math::boundedLinearInterpolation(1, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(-10, Math::boundedLinearInterpolation(10, -10, 10, 10, -10));
    EXPECT_FLOAT_EQ(-10, Math::boundedLinearInterpolation(11, -10, 10, 10, -10));

    // points (-8, 2) and (4, 5), i.e. f(x) = 0.25 * clamp(x, -5, 0) + 4
    EXPECT_FLOAT_EQ(2, Math::boundedLinearInterpolation(-8, -8, 4, 2, 5));
    EXPECT_FLOAT_EQ(2, Math::boundedLinearInterpolation(-9, -8, 4, 2, 5));
    EXPECT_FLOAT_EQ(2.25, Math::boundedLinearInterpolation(-7, -8, 4, 2, 5));
    EXPECT_FLOAT_EQ(4, Math::boundedLinearInterpolation(0, -8, 4, 2, 5));
    EXPECT_FLOAT_EQ(5, Math::boundedLinearInterpolation(4, -8, 4, 2, 5));
    EXPECT_FLOAT_EQ(5, Math::boundedLinearInterpolation(5, -8, 4, 2, 5));
}

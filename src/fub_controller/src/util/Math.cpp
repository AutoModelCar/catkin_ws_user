#include "Math.h"

#include <assert.h>
#include <math.h>

namespace fub {
namespace controller {
namespace util {

double Math::boundedLinearInterpolation(double v, double lowerBound, double upperBound, double lowerValue, double upperValue)
{
    assert(lowerBound < upperBound);

	if (v <= lowerBound) {
		return lowerValue;
	}
	else if (v >= upperBound) {
		return upperValue;
	}
	else {
		return ((((v - lowerBound) / (upperBound - lowerBound)) * (upperValue - lowerValue))  + lowerValue);
	}
}


void Math::normalizeAngle(double & angle)
{
	if (angle <= (-M_PI)) {
		angle += (2 * M_PI);
	}
	else if (angle > M_PI) {
		angle -= (2 * M_PI);
	}
}

}
}
}

#pragma once

namespace fub {
namespace controller {
namespace util {

class Math {
public:
    /** Linearly interpolate within a range. Assume the two points
     ** P1(lowerBound, lowerValue) and P2(upperBound, upperValue)
     ** define a linear function f. Given a value v, clamp v to
     ** [lowerBound, upperBound] and return f( clamp(v) ).
     **
     ** @param v              The value to calculate the bounded interpolation for.
     ** @param lowerBound     The lower bound of the interpolation range.
     ** @param upperBound     The upper bound of the interpolation range.
     ** @param lowerValue     The lower value (at lowerBound)
     ** @param upperValue     The upper value (at upperBound)
     **
     ** @return the interpolated value f(x), where x is v clamped to [lowerBound,
     **         upperBound] and f is the linear function described by the two
     **         points (lowerBound, lowerValue) and (upperBound, upperValue)
     **
     ** @pre lowerBound < upperBound
     */
	static double boundedLinearInterpolation(double v, double lowerBound, double upperBound, double lowerValue, double upperValue);

    /** Normalizes the angle to be within ]-pi, pi].
	 **
	 ** @param angle[in, out]  angle
	 */
	static void normalizeAngle(double & angle);
};

}
}
}

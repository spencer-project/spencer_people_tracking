#include <srl_laser_features/features/feature33.h>

namespace srl_laser_features {

void Feature33::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector2d();

	// distance between first point of segment and preceding point
	if( isnan(segment.precedingPoint(0)) || isnan(segment.precedingPoint(1)) ) {
		// No preceding point exists
		result(0) = -1.0;
	}
	else {
		result(0) = (segment.points.front() - segment.precedingPoint).norm();
	}

	// distance between last point of segment and succeeding point
	if( isnan(segment.succeedingPoint(0)) || isnan(segment.succeedingPoint(1)) ) {
		// No succeeding point exists
		result(1) = -1.0;
	}
	else {
		result(1) = (segment.points.back() - segment.succeedingPoint).norm();
	}
}

} // end of namespace srl_laser_features

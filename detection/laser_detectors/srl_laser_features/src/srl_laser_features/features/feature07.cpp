#include <srl_laser_features/features/feature07.h>

namespace srl_laser_features {

void Feature07::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector3d::Zero();
	
	const size_t numPoints = segment.points.size();
	if (numPoints > 2) {
		// the points are in Cartesian coordinates but ordered by their polar angle
		double distances[numPoints - 1];

		// open contour
		for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
			distances[pIndex] = (segment.points[pIndex] - segment.points[pIndex+1]).norm();
			result(0) += distances[pIndex];
		}

		// perimeter
		result(1) = result(0) + (segment.points.front() - segment.points.back()).norm();

		// regularity
		double mean = result(0) / (numPoints - 1);
		for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
			double diff = distances[pIndex] - mean;
			result(2) += diff * diff;
		}
		result(2) = sqrt(result(1) / (numPoints - 2));
	}
}

} // end of namespace srl_laser_features

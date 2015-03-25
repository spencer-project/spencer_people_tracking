#include <srl_laser_features/features/feature04.h>

namespace srl_laser_features {

void Feature04::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector1d::Zero();

	const size_t numPoints = segment.points.size();
	if (numPoints > 1) {
		for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
			result(0) += (segment.median - segment.points[pIndex]).squaredNorm();
		}
		result(0) = sqrt(result(0) / numPoints);
	}	
}

} // end of namespace srl_laser_features

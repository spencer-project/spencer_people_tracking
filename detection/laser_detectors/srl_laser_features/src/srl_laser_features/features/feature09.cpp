#include <srl_laser_features/features/feature09.h>

namespace srl_laser_features {

void Feature09::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector1d::Zero();
	const size_t numPoints = segment.points.size();

	if (numPoints > 2) {
		double std_x = 0.0;
		double std_y = 0.0;

		for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
			double dx = segment.mean(0) - segment.points[pIndex](0);
			double dy = segment.mean(1) - segment.points[pIndex](1);
			std_x += dx * dx;
			std_y += dy * dy;
		}

		std_x = sqrt(std_x / (numPoints - 1));
		std_y = sqrt(std_y / (numPoints - 1));

		double min, max;
		if (std_x < std_y) {
			min = std_x;
			max = std_y;
		} 
		else {
			min = std_y;
			max = std_x;
		}

		result(0) = (1.0 + min) / (1.0 + max);
	}
}

} // end of namespace srl_laser_features

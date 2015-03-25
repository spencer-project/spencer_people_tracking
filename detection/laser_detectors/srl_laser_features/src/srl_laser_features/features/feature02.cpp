#include <srl_laser_features/features/feature02.h>

namespace srl_laser_features {

void Feature02::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector2d::Zero();
	if (segment.points.size() > 1 && segment.ranges.size() == segment.points.size()) {
		std::vector<double> differences;

		// mean
		for (size_t pIndex = 0; pIndex < segment.points.size() - 1; ++pIndex) {
			double difference = fabs(segment.ranges[pIndex] - segment.ranges[pIndex + 1]);
			result(0) += difference;
			differences.push_back(difference);
		}
		
		result(0) /= differences.size();
		
		// std
		if (differences.size() > 2) {
			for (unsigned int dIndex = 0; dIndex < differences.size(); ++dIndex) {
				double val = differences[dIndex] - result(0);
				result(1) += val * val;
			}
			result(1) = sqrt(result(1) / (differences.size() - 1));
		}
	} 
	else {
		result.setConstant(2, -1.0);
	}
}

} // end of namespace srl_laser_features

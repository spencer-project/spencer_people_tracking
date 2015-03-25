#include <srl_laser_features/features/feature14.h>
#include <float.h>


namespace srl_laser_features {

void Feature14::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::VectorXd::Zero(16);
	
	const size_t numRanges = segment.ranges.size();
	if (numRanges > 1) {		
		double noise[15];
		bool inMimima[15];

		for (size_t index = 1; index < 15; ++index) {
			noise[index] = 0.01 * index;
			inMimima[index] = false;
		}

		double min = DBL_MAX, max = -1.0, last = DBL_MAX;

		for (size_t pIndex = 0; pIndex < numRanges; ++pIndex) {
			const double range = segment.ranges[pIndex];
			min = std::min(min, range);
			max = std::max(max, range);
			
			for (size_t index = 1; index < 15; ++index) {
				if (range < last - noise[index]) {
					if (!inMimima[index]) {
						++result(index);
					}
				} else {
					inMimima[index] = false;
				}
			}
			last = range;
		}

		result(0) = max - min;
		if (max > 0.0) {
			result(15) = min / max;
		}
	}
}

} // end of namespace srl_laser_features

#include <srl_laser_features/features/feature13.h>

namespace srl_laser_features {

void Feature13::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::VectorXd::Zero(1);
	
	const size_t numPoints = segment.points.size();
	if (numPoints > 2) {
		for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
			result(0) += ( segment.points[pIndex  ](0) * segment.points[pIndex+1](1) ) 
					   - ( segment.points[pIndex+1](0) * segment.points[pIndex  ](1) );
		}

		// close polygon
		result(0) += ( segment.points.back() (0) * segment.points.front()(1) ) 
				   - ( segment.points.front()(0) * segment.points.back() (1) );

		result(0) *= 0.5;
		result(0) = fabs(result(0));
	}
}

} // end of namespace srl_laser_features

#include <srl_laser_features/features/feature01.h>

namespace srl_laser_features {

void Feature01::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector1d::Zero();
    result(0) = segment.points.size();
}

} // end of namespace srl_laser_features

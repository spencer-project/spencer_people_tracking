#include <srl_laser_features/features/feature12.h>

namespace srl_laser_features {

void Feature12::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector1d::Zero();
	result(0) = segment.mean.norm(); // assumes sensor in origin at (0,0)
}

} // end of namespace srl_laser_features

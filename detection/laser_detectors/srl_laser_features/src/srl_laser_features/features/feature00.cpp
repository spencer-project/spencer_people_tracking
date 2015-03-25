#include <srl_laser_features/features/feature00.h>
#include <cstdlib>

namespace srl_laser_features {

Feature00::Feature00() : Feature()
{
	srand(666);
}

void Feature00::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector1d::Zero();

	if ( rand() % RAND_MAX > RAND_MAX/2 ) {
		result(0) = 1;
	} else {
		result(0) = 0;
	}
}

} // end of namespace srl_laser_features
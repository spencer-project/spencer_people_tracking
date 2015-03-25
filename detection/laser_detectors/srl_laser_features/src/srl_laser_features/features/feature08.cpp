#include <srl_laser_features/features/feature08.h>
#include <ros/console.h>
#include <algorithm>
#include <exception>

namespace srl_laser_features {

void Feature08::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector2d::Zero();
	
	const size_t numPoints = segment.points.size();
	if (numPoints > 2) {
		// the points are in Cartesian coordinates but ordered by their polar angle
		std::vector<double> d(3);
		double dx0, dx1, dx2;
		double dy0, dy1, dy2;
		double area;
		double denom;
		double div;

		for (size_t pIndex = 1; pIndex < numPoints - 1; ++pIndex) {
			d[0] = (segment.points[pIndex-1] - segment.points[pIndex  ]).norm();
			d[1] = (segment.points[pIndex  ] - segment.points[pIndex+1]).norm();
			d[2] = (segment.points[pIndex-1] - segment.points[pIndex+1]).norm();	

			// mean angular difference
			div = d[0] * d[1];
			if (div > 0) {
				// scalar product
				double sc_prd = dx0 * dx1 + dy0 * dy1;
				double res = sc_prd / div;

				//~ avoids nasty rounding errors of the compiler
				if (res > 0.0) {
					res -= 1e-10;
				}
				if (res < 0.0) {
					res += 1e-10;
				}

				if (fabs(res) > 1.0) {
					ROS_FATAL("Feature08::%s: memory corruption!", __func__);
					std::terminate();
				}
				result(1) += acos(res);
			}

			// mean curvature
			std::sort(d.begin(), d.end());

			// numerically stable Hero's formula
			// + 1e-10 is due to aligned points
			area = (d[0] + d[1] + d[2]) * (d[2] - (d[0] - d[1])) * (d[2] + (d[0] - d[1])) * (d[0] + (d[1] - d[2])) + 1e-10;
			area = 0.25 * sqrt(area);
			denom = d[0] * d[1] * d[2] + 1e-10;
			result(0) += (4 * area) / denom;
		}

		result(1) /= (numPoints - 2);
		result(0) /= numPoints;
	}
}

} // end of namespace srl_laser_features

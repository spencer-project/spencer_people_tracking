/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2006-2012, Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
*    Oscar Martinez, Autonomous Intelligent Systems, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
		double dx0, dx1;
		double dy0, dy1;
		double area;
		double denom;
		double div;

		for (size_t pIndex = 1; pIndex < numPoints - 1; ++pIndex) {
			d[0] = (segment.points[pIndex-1] - segment.points[pIndex  ]).norm();
			d[1] = (segment.points[pIndex  ] - segment.points[pIndex+1]).norm();
			d[2] = (segment.points[pIndex-1] - segment.points[pIndex+1]).norm();

			dx0 = segment.points[pIndex-1][0] - segment.points[pIndex][0];
			dy0 = segment.points[pIndex-1][1] - segment.points[pIndex][1];
			dx1 = segment.points[pIndex][0] - segment.points[pIndex+1][0];
			dy1 = segment.points[pIndex][1] - segment.points[pIndex+1][1];

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

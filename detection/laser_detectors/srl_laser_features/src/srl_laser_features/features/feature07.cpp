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

#include <srl_laser_features/features/feature07.h>

namespace srl_laser_features {

void Feature07::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector3d::Zero();

	const size_t numPoints = segment.points.size();
	if (numPoints > 2) {
		// the points are in Cartesian coordinates but ordered by their polar angle
		double distances[numPoints - 1];

		// open contour
		for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
			distances[pIndex] = (segment.points[pIndex] - segment.points[pIndex+1]).norm();
			result(0) += distances[pIndex];
		}

		// perimeter
		result(1) = result(0) + (segment.points.front() - segment.points.back()).norm();

		// regularity
		double mean = result(0) / (numPoints - 1);
		for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
			double diff = distances[pIndex] - mean;
			result(2) += diff * diff;
		}
		result(2) = sqrt(result(1) / (numPoints - 2));
	}
}

} // end of namespace srl_laser_features

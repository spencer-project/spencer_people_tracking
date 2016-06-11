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

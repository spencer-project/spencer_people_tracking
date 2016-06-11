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

#include <srl_laser_features/features/feature05.h>
#include <float.h>

namespace srl_laser_features
{

Feature05::Feature05(bool extended) : Feature(), m_extended(extended)
{
}

void Feature05::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
    result = Eigen::VectorXd::Zero(getNDimensions());

    const size_t numPoints = segment.points.size();
    if (numPoints > 1) {
        // width
        result(0) = (segment.points.back() - segment.points.front()).norm();

        if (m_extended) {
            // min and max distance between points
            result(1) = DBL_MAX;
            result(2) = 0.0;

            for (size_t pIndex = 0; pIndex < numPoints - 1; ++pIndex) {
                double dist = (segment.points[pIndex] - segment.points[pIndex + 1]).norm();
                if (dist < result(1)) result(1) = dist;
                if (dist > result(2)) result(2) = dist;
            }

            // ratio between min and max distance between points
            if (result(2) != 0.0) {
                result(3) = result(1) / result(2);
            }
        }
    }
    else {
        result.setConstant(getNDimensions(), -1.0);
    }
}

} // end of namespace srl_laser_features

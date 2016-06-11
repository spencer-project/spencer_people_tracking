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

#include <srl_laser_features/features/feature10.h>
#include <Eigen/QR>
#include <float.h>


namespace srl_laser_features
{

Feature10::Feature10(bool extended) : Feature(), m_extended(extended)
{
}

void Feature10::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
    result = Eigen::VectorXd::Zero(getNDimensions());

    const size_t numPoints = segment.points.size();
    if (numPoints >= 2) {
        double px[numPoints];
        double py[numPoints];

        // copy
        for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
            px[pIndex] = segment.points[pIndex](0);
            py[pIndex] = segment.points[pIndex](1);
        }

        double m, q, diff;
        result(0) = 0.0;
        if (m_extended) {
            result(1) = 0.0;
            result(2) = DBL_MAX;
            result(3) = 0.0;
        }
        fitLine(numPoints, px, py, m, q);

        // check for vertical line
        if (fabs(m) > 10e+10) {
            // residual sum
            double diff;
            for (size_t i = 0; i < numPoints; ++i) {
                diff = segment.points[i](0) - segment.mean(0);
                result(0) += diff * diff;
                if (m_extended) {
                    if (diff < result(2)) {
                        result(2) = diff;
                    }
                    if (diff > result(3)) {
                        result(3) = diff;
                    }
                }
            }
        }
        else {
            // residual sum
            for (size_t i = 0; i < numPoints; ++i) {
                diff = m * segment.points[i](0) + q - segment.points[i](1);
                result(0) += diff * diff;
                if (m_extended) {
                    if (diff < result(2)) {
                        result(2) = diff;
                    }
                    if (diff > result(3)) {
                        result(3) = diff;
                    }
                }
            }
        }

        if (m_extended) {
            result(1) = result(0) / numPoints;
            if (result(3) != 0.0) {
                result(4) = result(2) / result(3);
            }
        }
    }
    else {
        result.setConstant(getNDimensions(), -1.0);
    }
}

void Feature10::fitLine(int n, double* xData, double* yData, double& m, double& q)
{
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd B(n);
    Eigen::VectorXd C(2);

    // fill A and B
    for (int i = 0; i < n; i++) {
        A(i, 0) = xData[i];
        A(i, 1) = 1.0;
        B(i) = yData[i];
    }

    // solve
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> QR(A);
    C = QR.solve(B);
    m = C(0);
    q = C(1);
}

} // end of namespace srl_laser_features

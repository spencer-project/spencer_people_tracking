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

#include <srl_laser_features/features/feature06.h>

#include <Eigen/LU>
#include <Eigen/QR>
#include <float.h>

#define MATRIX_LN_EPS -1e8


namespace srl_laser_features {

Feature06::Feature06(bool extended) : Feature(), m_extended(extended)
{
}

void Feature06::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::VectorXd::Zero(getNDimensions());

	const size_t numPoints = segment.points.size();
	if (numPoints > 2) {

		double px[numPoints];
		double py[numPoints];

		// copy
		for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
			px[pIndex] = segment.points[pIndex](0);
			py[pIndex] = segment.points[pIndex](1);
		}

		double xc, yc, rc;
		fitCircle(numPoints, px, py, xc, yc, rc);

		result(1) = rc;
		if (m_extended) {
			result(3) = DBL_MAX;
			result(4) = 0.0;
		}

		// residual sum
		Eigen::Vector2d centerOfCircle(xc, yc);

		double residual;
		for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
			double dist = (centerOfCircle - segment.points[pIndex]).norm();

			if (m_extended) {
				if (dist < result(3)) {
					result(3) = dist;
				}
				if (dist > result(4)) {
					result(4) = dist;
				}
			}

			residual = rc - dist;
			result(0) += residual * residual;
		}

		if (m_extended) {
			result(2) = result(0) / (numPoints - 1);
			if (result(4) != 0.0) {
				result(5) = result(3) / result(4);
			}
		}
	}
}


int Feature06::fitCircle(int n, double *x_vec, double *y_vec, double& xc, double& yc, double& r)
{
	int i;

	if (n > 3) {
		Eigen::MatrixXd A(n, 3);
		Eigen::MatrixXd X(3, 1);
		Eigen::MatrixXd B(n, 3);
		Eigen::MatrixXd P(3, n);
		Eigen::MatrixXd T(3, 3);

		// We want the overdetermined linear equation system Ax = b
		// Fill in matrix A
		for (i = 0; i < n; i++) {
			A(i, 0) = -2 * x_vec[i];
			A(i, 1) = -2 * y_vec[i];
			A(i, 2) = 1.0;
		}

		// Fill in vector b
		for (i = 0; i < n; i++) {
			B(i, 0) = -(x_vec[i] * x_vec[i] + y_vec[i] * y_vec[i]);
		}

		// Now we have the equation system with n equations and 3 unknowns.
		// The LSQ-solution is given by the pseudo-inverse:
		P = A.transpose();
		T = P * A;
		Eigen::FullPivLU<Eigen::MatrixXd> lu(T);
		if (lu.isInvertible()) {
			if (log(lu.determinant()) > MATRIX_LN_EPS) {
				X = lu.inverse() * P * B;
				// Extract circle center and radius
				xc = X(0, 0);
				yc = X(1, 0);
				r = sqrt(-X(2, 0) + xc * xc + yc * yc);
				return 1;
			}
			else {
				// Points badly conditioned (n > 3)
				return -3;
			}
		}
		else {
			// try QR
			Eigen::FullPivHouseholderQR<Eigen::MatrixXd> QR(A);
			X = QR.solve(B);

			// Extract circle center and radius
			xc = X(0, 0);
			yc = X(1, 0);
			r = sqrt(-X(2, 0) + xc * xc + yc * yc);
			return 1;
		}
	}
	else if (n == 3) {
		double a, b, c, d, e, f, g;

		a = x_vec[1] - x_vec[0];
		b = y_vec[1] - y_vec[0];
		c = x_vec[2] - x_vec[0];
		d = y_vec[2] - y_vec[0];
		e = a * (x_vec[0] + x_vec[1]) + b * (y_vec[0] + y_vec[1]);
		f = c * (x_vec[0] + x_vec[2]) + d * (y_vec[0] + y_vec[2]);
		g = 2 * (a * (y_vec[2] - y_vec[1]) - b * (x_vec[2] - x_vec[1]));

		if (g != 0.0) {
			xc = (d * e - b * f) / g;
			yc = (a * f - c * e) / g;
			r = sqrt((x_vec[0] - xc) * (x_vec[0] - xc) + (y_vec[0] - yc) * (y_vec[0] - yc));
			return 2;
		}
		else {
			// Points badly conditioned (n == 3)
			return -2;
		}
	}
	else {
		// Too few points
		xc = 0.0;
		yc = 0.0;
		r = -1.0;
		return -1;
	}
}

} // end of namespace srl_laser_features

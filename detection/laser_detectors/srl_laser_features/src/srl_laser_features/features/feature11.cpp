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

#include <srl_laser_features/features/feature11.h>
#include <ros/console.h>
#include <float.h>


namespace srl_laser_features {

void Feature11::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	result = Eigen::Vector2d::Zero();

	const size_t numPoints = segment.points.size();
	if (numPoints > 1) {
		double px[numPoints];
		double py[numPoints];

		// copy
		for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
			px[pIndex] = segment.points[pIndex](0);
			py[pIndex] = segment.points[pIndex](1);
		}

		double sx[numPoints];
		double sy[numPoints];

		if (fitBSpline(numPoints, px, py, 2, numPoints, sx, sy)) {
			result(0) = getErrorBSpline(numPoints, px, py, numPoints, sx, sy);
		}
		else {
			result(0) = -1.0;
		}

		if (fitBSpline(numPoints, px, py, 3, numPoints, sx, sy)) {
			result(1) = getErrorBSpline(numPoints, px, py, numPoints, sx, sy);
		}
		else {
			result(1) = -1.0;
		}
	}
}

void Feature11::compute_intervals(int *u, int n, int t) // figure out the knots
{
	int j;

	for (j = 0; j <= n + t; j++) {
		if (j < t)
			u[j] = 0;
		else if ((t <= j) && (j <= n))
			u[j] = j - t + 1;
		else if (j > n)
			u[j] = n - t + 2; // if n-t=-2 then we're screwed, everything goes to 0
	}
}

double Feature11::blend(int k, int t, int *u, double v) // calculate the blending value
{
	double value;

	if (t == 1) // base case for the recursion
	{
		if ((u[k] <= v) && (v < u[k + 1]))
			value = 1;
		else
			value = 0;
	} else {
		if ((u[k + t - 1] == u[k]) && (u[k + t] == u[k + 1])) // check for divide by zero
			value = 0;
		else if (u[k + t - 1] == u[k]) // if a term's denominator is zero,use just the other
			value = (u[k + t] - v) / (u[k + t] - u[k + 1]) * blend(k + 1, t - 1, u, v);
		else if (u[k + t] == u[k + 1])
			value = (v - u[k]) / (u[k + t - 1] - u[k]) * blend(k, t - 1, u, v);
		else
			value = (v - u[k]) / (u[k + t - 1] - u[k]) * blend(k, t - 1, u, v) + (u[k + t] - v)
					/ (u[k + t] - u[k + 1]) * blend(k + 1, t - 1, u, v);
	}
	return value;
}

void Feature11::compute_point(int *u, int n, int t, double v, double* controlx, double* controly, double& outputx, double& outputy)
{
	int k;
	double temp;
	// initialize the variables that will hold our outputted point
	outputx = 0.0;
	outputy = 0.0;

	for (k = 0; k <= n; k++) {
		temp = blend(k, t, u, v); // same blend is used for each dimension coordinate
		outputx = outputx + controlx[k] * temp;
		outputy = outputy + controly[k] * temp;
	}
}

bool Feature11::fitBSpline(int n, double* xData, double* yData, int t, int num_output, double* sx, double* sy)
{
	// init
	++t;
	--n; // !!!
	if (n + 2 <= t) {
		// ROS_WARN("Not enough control points for this degree");
		return false;
	}
	double increment, interval;
	int output_index;

	int u[n + t + 1];

	compute_intervals(u, n, t);

	if (num_output > 1) {
		increment = (double) (n - t + 2) / (num_output - 1); // how much parameter goes up each time
	} else {
		increment = 0; // stupid user wants only one frame
	}

	interval = 0.0;
	for (output_index = 0; output_index < num_output - 1; output_index++) {
		compute_point(u, n, t, interval, xData, yData, sx[output_index], sy[output_index]);
		interval = interval + increment; // increment our parameter
	}

	// put in the last point
	sx[num_output - 1] = xData[n];
	sy[num_output - 1] = yData[n];

	return true;

}

double Feature11::getErrorBSpline(int n, double* xData, double* yData, int m, double* sx, double* sy)
{
	double RSS = 0;

	//~ normalized error in control points
	for (int i = 0; i < n; i++) {
		double best_dist = DBL_MAX;
		for (int j = 0; j < m; j++) {
			double dx = xData[i] - sx[i];
			double dy = yData[i] - sy[i];
			double v = hypot(dx, dy);
			if (v < best_dist) {
				best_dist = v;
			}
		}
		RSS += best_dist;
	}
	return (RSS / n);
}

} // end of namespace srl_laser_features

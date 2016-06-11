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

#ifndef SRL_LASER_FEATURES_FEATURE11_H_
#define SRL_LASER_FEATURES_FEATURE11_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Error of fitted quadratic b-spline and cubic b-spline.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature11: public Feature
{
public:
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Error of fitted quadratic b-spline.";
		}
		if (dimension == 1) {
			return "Error of fitted cubic b-spline.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 2;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;

private:
	static bool fitBSpline(int n, double* xData, double* yData, int t, int num_output, double* sx, double* sy);
	static double getErrorBSpline(int n, double* xData, double* yData, int m, double* sx, double* sy);

	static void compute_intervals(int *u, int n, int t);
	static double blend(int k, int t, int *u, double v);
	static void compute_point(int *u, int n, int t, double v, double* controlx, double* controly, double& outputx, double& outputy);
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE11_H_

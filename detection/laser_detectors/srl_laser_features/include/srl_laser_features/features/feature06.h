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

#ifndef SRL_LASER_FEATURES_FEATURE06_H_
#define SRL_LASER_FEATURES_FEATURE06_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Circularity and radius of fitted circle.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature06: public Feature
{
public:
	Feature06(bool extended = true);

	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Circularity.";
		}
		if (dimension == 1) {
			return "Radius of fitted circle.";
		}
		if (dimension == 2) {
			return "Variance of distance from points to center of circle.";
		}
		if (dimension == 3) {
			return "Minimal difference to radius.";
		}
		if (dimension == 4) {
			return "Maximal difference to radius.";
		}
		if (dimension == 5) {
			return "Ratio between min and max difference.";
		}
		if (dimension == 6) {
			return "Area of rectangles to fitted circle center.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		if (m_extended) {
			return 7;
		}
		else {
			return 2;
		}
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;

private:
	/// Fit circle to points in Cartesian coordinates
	/// Fits a circle in the least squares sense to points given in Cartesian coordinates minimizing
	/// algebraic distances. The problem has a fast and geometrically good solution when parametrized
	/// as (xc, yx, xc^2+yc^2-r^2) for the vector of unknowns. x_vec and y_vec are array of length n
	/// where n >= 3. If n == 3 the circle that goes exactly through the three points is calculated.
	/// The function returns the circle center xc, yc and the circle radius r.
	static int fitCircle(int n, double* xData, double* yData, double& xc, double& yc, double& radius);

	bool m_extended;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE06_H_

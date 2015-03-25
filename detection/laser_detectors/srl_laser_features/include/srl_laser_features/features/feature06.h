/*!
 \brief Header file of the Feature06 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
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
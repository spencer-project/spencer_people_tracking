/*!
 \brief Header file of the Feature11 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
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


/*!
 \brief Header file of the Feature10 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE10_H_
#define SRL_LASER_FEATURES_FEATURE10_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Linearity.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature10: public Feature
{
public: 
	Feature10(bool extended = true);
	
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Linearity.";
		}
		if (dimension == 1) {
			return "Linearity mean error.";
		}
		if (dimension == 2) {
			return "Linearity min error.";
		}
		if (dimension == 3) {
			return "Linearity max error.";
		}
		if (dimension == 4) {
			return "Linearity ratio of min and max error.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		if (m_extended) {
			return 5;
		} 
		else {
			return 1;
		}
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;

private:
	static void fitLine(int n, double* xData, double* yData, double& m, double& q);

	bool m_extended;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE10_H_
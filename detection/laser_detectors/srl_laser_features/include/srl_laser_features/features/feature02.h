/*!
 \brief Header file of the Feature02 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE02_H_
#define SRL_LASER_FEATURES_FEATURE02_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Mean and variance of difference in range of consecutive beams.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature02: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());

		if (dimension == 0) {
			return "Mean of difference in range of consecutive beams.";
		}
		if (dimension == 1) {
			return "Variance of difference in range of consecutive beams.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 2;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE02_H_

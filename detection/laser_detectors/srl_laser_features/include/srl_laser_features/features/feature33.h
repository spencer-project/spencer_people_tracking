/*!
 \brief Header file of the Feature33 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE33_H_
#define SRL_LASER_FEATURES_FEATURE33_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Jump distance to previous and next segment (defined on first and last laser end points).
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature33: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Jump distance to previous segment.";
		}
		if (dimension == 1) {
			return "Jump distance to next segment.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 2;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE33_H_
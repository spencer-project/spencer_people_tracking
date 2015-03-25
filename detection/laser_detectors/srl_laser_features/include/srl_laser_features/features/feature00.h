/*!
 \brief Header file of the Feature00 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE00_H_
#define SRL_LASER_FEATURES_FEATURE00_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Throwing a coin. If this feature is one of the best, either something is wrong (very likely) or the classes are not distinguishable!
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature00: public Feature
{
public: 
	Feature00();
	
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		return "Throw a coin.";
	}

	inline unsigned int getNDimensions() const
	{
		return 1;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE00_H_

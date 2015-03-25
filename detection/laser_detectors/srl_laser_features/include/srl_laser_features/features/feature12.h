/*!
 \brief Header file of the Feature12 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE12_H_
#define SRL_LASER_FEATURES_FEATURE12_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Distance to sensor.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature12: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		return "Distance.";
	}

	inline unsigned int getNDimensions() const
	{
		return 1;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE12_H_
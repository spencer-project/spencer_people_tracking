/*!
 \brief Header file of the Feature01 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE01_H_
#define SRL_LASER_FEATURES_FEATURE01_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Number of laser end points in the Segment.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature01: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		return "Number of laser end points.";
	}

	inline unsigned int getNDimensions() const
	{
		return 1;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE01_H_

/*!
 \brief Header file of the Feature09 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE09_H_
#define SRL_LASER_FEATURES_FEATURE09_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Aspect ratio of max and min standard deviation.
 * \f$ F09 = \frac{\max(std_x,std_y)}{\min(std_x,std_y)} \f$
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature09: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		return "Aspect ratio.";
	}

	inline unsigned int getNDimensions() const
	{
		return 1;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE09_H_
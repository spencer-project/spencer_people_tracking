/*!
 \brief Header file of the Feature08 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE08_H_
#define SRL_LASER_FEATURES_FEATURE08_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Use convex hull to calculate:
 * mean curvature and mean angular difference on convex hull border
 * and perimeter.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature08: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Mean curvature.";
		}
		if (dimension == 1) {
			return "Mean angular difference.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 2;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE08_H_
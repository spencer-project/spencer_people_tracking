/*!
 \brief Header file of the Feature07 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE07_H_
#define SRL_LASER_FEATURES_FEATURE07_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Use convex hull to calculate:
 * Boundary length (open contour),
 * and the perimeter,
 * and regularity of the contour.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature07: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Boundary length.";
		}
		if (dimension == 1) {
			return "Perimeter.";
		}
		if (dimension == 2) {
			return "Contour regularity.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 3;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE07_H_
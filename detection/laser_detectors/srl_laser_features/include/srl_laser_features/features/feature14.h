/*!
 \brief Header file of the Feature14 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE14_H_
#define SRL_LASER_FEATURES_FEATURE14_H_

#include <srl_laser_features/features/feature.h>
#include <cstdio>


namespace srl_laser_features {

/**
 * Difference of shortest and longest beam and number of local minima.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature14: public Feature
{
public: 
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Difference of longest and shortest beam.";
		}

		// dimensions 1 to 14 reflect number of minima, calculated with noise 0.01 * dimension
		if (dimension < 15) {
			char text[128];
			sprintf(text, "Number of local minima (noise %.2f).", 0.01 * dimension);
			return text;
		}

		if (dimension == 15) {
			return "Ratio between shortest and longest beam.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		return 16;
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE14_H_
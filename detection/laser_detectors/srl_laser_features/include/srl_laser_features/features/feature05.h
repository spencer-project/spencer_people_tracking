/*!
 \brief Header file of the Feature05 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE05_H_
#define SRL_LASER_FEATURES_FEATURE05_H_

#include <srl_laser_features/features/feature.h>


namespace srl_laser_features {

/**
 * Width of the Segment (distance between first and last laser end point).
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature05: public Feature
{
public: 
	Feature05(bool extended = true);
	
	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());
		if (dimension == 0) {
			return "Width.";
		}
		if (dimension == 1) {
			return "Min distance between points.";
		}
		if (dimension == 2) {
			return "Max distance between points.";
		}
		if (dimension == 3) {
			return "Ratio between min and max distance between points.";
		}
	}

	inline unsigned int getNDimensions() const
	{
		if (m_extended) {
			return 4;
		} 
		else {
			return 1;
		}
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;

private:
	bool m_extended;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE05_H_
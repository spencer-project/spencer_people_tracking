/*!
 \brief Header file of the Feature class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE_H_
#define SRL_LASER_FEATURES_FEATURE_H_

#include <srl_laser_features/segment.h>
#include <Eigen/Core>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace Eigen {
	typedef Matrix<double, 1, 1> Vector1d;
}


namespace srl_laser_features {

/// A single dimension of a Feature (which may have multiple of them), identified by a unique string descriptor.
typedef std::string FeatureDimension;
typedef std::vector<FeatureDimension> FeatureDimensions;

/// Abstract Feature class.
/// @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
class Feature
{
public:
	/// Destructor. 
	virtual ~Feature()
	{}

	/// Get a short description of the feature. 
	virtual const FeatureDimension getDescription(unsigned int dimension) const = 0;

	/// Get the number of dimensions of this feature. 
	virtual unsigned int getNDimensions() const = 0;

	/// Calculate feature values. 
	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const = 0;

	// Typedefs
	typedef boost::shared_ptr<Feature> Ptr;
};

/// Typedefs for readability
typedef std::vector<Feature::Ptr> Features;


}  // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE_H_

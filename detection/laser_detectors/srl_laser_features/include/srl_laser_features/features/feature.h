/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2006-2012, Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
*    Oscar Martinez, Autonomous Intelligent Systems, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

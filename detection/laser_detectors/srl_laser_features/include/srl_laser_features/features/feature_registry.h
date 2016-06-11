/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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

#ifndef SRL_LASER_FEATURES_FEATURE_REGISTRY_H_
#define SRL_LASER_FEATURES_FEATURE_REGISTRY_H_

#include <srl_laser_features/features/feature.h>

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>


namespace srl_laser_features {


/// A registry of all available features, see cpp file for list of features
class FeatureRegistry
{
public:
    /// Constructor, initializes all features.
    FeatureRegistry();

    /// Returns a list of all available features.
    static const Features& getAllFeatures() {
        return getInstance()->m_features;
    }

    /// Returns a list of all available feature dimensions (as string descriptors) of all features.
    static const FeatureDimensions& getAllFeatureDimensions() {
        return getInstance()->m_featureDimensions;
    }

private:
    typedef boost::shared_ptr<FeatureRegistry> Ptr;
    static Ptr instance;

    static Ptr getInstance() {
        if(!instance) {
            instance.reset(new FeatureRegistry);
        }
        return instance;
    }


    Features m_features;
    FeatureDimensions m_featureDimensions;
};

}  // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE_REGISTRY_H_

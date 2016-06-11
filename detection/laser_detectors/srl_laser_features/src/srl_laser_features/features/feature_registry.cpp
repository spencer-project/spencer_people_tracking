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

#include <srl_laser_features/features/feature_registry.h>

#include <srl_laser_features/features/feature00.h>
#include <srl_laser_features/features/feature01.h>
#include <srl_laser_features/features/feature02.h>
#include <srl_laser_features/features/feature03.h>
#include <srl_laser_features/features/feature04.h>
#include <srl_laser_features/features/feature05.h>
#include <srl_laser_features/features/feature06.h>
#include <srl_laser_features/features/feature07.h>
#include <srl_laser_features/features/feature08.h>
#include <srl_laser_features/features/feature09.h>
#include <srl_laser_features/features/feature10.h>
#include <srl_laser_features/features/feature11.h>
#include <srl_laser_features/features/feature12.h>
#include <srl_laser_features/features/feature13.h>
#include <srl_laser_features/features/feature14.h>
#include <srl_laser_features/features/feature15.h>
#include <srl_laser_features/features/feature33.h>


namespace srl_laser_features {

FeatureRegistry::Ptr FeatureRegistry::instance;

// All available features are initialized here.
FeatureRegistry::FeatureRegistry() {
    // Register features
    Feature* features[] = {
        new Feature00,
        new Feature01,
        new Feature02,
        new Feature03,
        new Feature04,
        new Feature05,
        new Feature06,
        new Feature07,
        new Feature08,
        new Feature09,
        new Feature10,
        new Feature11,
        new Feature12,
        new Feature13,
        new Feature14,
        new Feature15,
        new Feature33
    };

    // Store features and feature dimension descriptors
    for(size_t i = 0; i < sizeof(features) / sizeof(features[0]); i++) {
        Feature* feature = features[i];
        m_features.push_back( Feature::Ptr(feature) );

        for(size_t dim = 0; dim < feature->getNDimensions(); dim++) {
            m_featureDimensions.push_back( feature->getDescription(dim) );
        }
    }
}


} // end of namespace srl_laser_features

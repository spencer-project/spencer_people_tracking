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

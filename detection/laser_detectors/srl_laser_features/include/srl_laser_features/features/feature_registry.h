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

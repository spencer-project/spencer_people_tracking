/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _PARAMS_H
#define _PARAMS_H

#include <ros/ros.h>


namespace srl_nnt {

/// Provides access to parameters read from a parameter server. This class is a singleton.
/// Use the static get<T>(parameterName, defaultValue) method to read the value of a parameter. 
class Params
{
public:
    /// Get value of the parameter with the specified name and default value, if no value has been configured.
    template<typename T> static T get(const std::string& parameterName, const T& defaultValue)
    {
        assert(s_instance);

        // Just a loose wrapper around the ROS parameter server API, uses local caching to avoid delays by having to contact the ROS master
        T value;
        if(!s_instance->m_privateNodeHandle.getParamCached(parameterName, value)) {
            value = defaultValue;
        }

        return value;
    }

    /// Set value of the parameter with the specified name and default value, if no value has been configured.
    template<typename T> static T set(const std::string& parameterName, const T& value)
    {
        assert(s_instance);

        // Just a loose wrapper around the ROS parameter server API, uses local caching to avoid delays by having to contact the ROS master
        s_instance->m_privateNodeHandle.setParam(parameterName, value);
        return value;
    }

    /// Constructor, uses the provided node handle to access the private parameter space.
    Params(ros::NodeHandle& privateNodeHandle);


private:
    /// ROS stuff
    ros::NodeHandle m_privateNodeHandle;

    /// Singleton instance of this class
    static Params* s_instance;
};


} // end of namespace srl_nnt


#endif // _PARAMS_H

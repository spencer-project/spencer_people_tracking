/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
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

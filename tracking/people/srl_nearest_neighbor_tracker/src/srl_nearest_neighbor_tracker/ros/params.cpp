#include <srl_nearest_neighbor_tracker/ros/params.h>

namespace srl_nnt {

Params* Params::s_instance = NULL;


Params::Params(ros::NodeHandle& privateNodeHandle)
    : m_privateNodeHandle(privateNodeHandle)
{
    assert(!s_instance);
    s_instance = this;
}

} // end of namespace srl_nnt
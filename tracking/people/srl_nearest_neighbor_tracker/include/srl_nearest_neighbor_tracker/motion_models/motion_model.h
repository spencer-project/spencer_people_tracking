/* Created on: Feb 06, 2015. Author: Fabian Girrbach */
#ifndef _MOTION_MODELS_MOTION_MODEL_H_
#define _MOTION_MODELS_MOTION_MODEL_H_

#include <srl_nearest_neighbor_tracker/base/defs.h>

namespace srl_nnt
{
/// Abstract class for a generic motion model for an EKF.
class MotionModel {
public:

    virtual const StateMatrix& A(const StateVector& x, const double deltaT) = 0;
    virtual const StateMatrix& getProcessNoiseQ(const double deltaT, const double processNoise) = 0;

    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<MotionModel> Ptr;
    typedef boost::shared_ptr<const MotionModel> ConstPtr;

    /// Return a deep copy of this filter state as a shared pointer
    virtual Ptr deepCopy() = 0;

protected:
    /// Transition matrix for motion model
    StateMatrix m_A;
    StateMatrix m_Q;
};

}


#endif /* _MOTION_MODELS_MOTION_MODEL_H_ */

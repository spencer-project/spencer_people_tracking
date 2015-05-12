/* Created on: Feb 06, 2015. Author: Fabian Girrbach */
#ifndef _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_
#define _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_

#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>
#include <ros/ros.h>


namespace srl_nnt {

class BrownianMotionModel: public MotionModel
{
public:
    virtual const StateMatrix& A (const StateVector& x, const double deltaT) {
        m_A = StateMatrix::Zero();
        m_A(STATE_X_IDX, STATE_X_IDX) = 1.0;
        m_A(STATE_Y_IDX, STATE_Y_IDX) = 1.0;
        return m_A;
    }

    virtual const StateMatrix& getProcessNoiseQ(const double deltaT, const double processNoise)
    {
        m_Q = StateMatrix::Zero();
        m_Q(0, 0) = deltaT * processNoise;
        m_Q(1, 1) = m_Q(0, 0);
        return m_Q;
    }


    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<BrownianMotionModel> Ptr;
    typedef boost::shared_ptr<const BrownianMotionModel> ConstPtr;

    /// Return a deep copy of this motion model as a shared pointer
    virtual MotionModel::Ptr deepCopy() {
        BrownianMotionModel* copy = new BrownianMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }
};

}


#endif /* _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_ */

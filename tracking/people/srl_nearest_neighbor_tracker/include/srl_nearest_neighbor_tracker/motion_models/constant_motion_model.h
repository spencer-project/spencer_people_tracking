/* Created on: Feb 06, 2015. Author: Fabian Girrbach */
#ifndef _MOTION_MODELS_CONSTANT_MOTION_MODEL_H_
#define _MOTION_MODELS_CONSTANT_MOTION_MODEL_H_

#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>
#include <ros/ros.h>

namespace srl_nnt {


class ConstantMotionModel: public MotionModel
{
public:
    virtual const StateMatrix& A (const StateVector& x, const double deltaT) {
        m_A = StateMatrix::Identity();
        m_A(STATE_X_IDX, STATE_VX_IDX) = deltaT;
        m_A(STATE_Y_IDX, STATE_VY_IDX) = deltaT;
        m_A(STATE_OMEGA_IDX, STATE_OMEGA_IDX) = 0;
        return m_A;
    }

    virtual const StateMatrix& getProcessNoiseQ(const double deltaT, const double processNoise)
    {
        m_Q = StateMatrix::Zero();
        m_Q(0, 0) = (deltaT * deltaT * deltaT) * processNoise / 3.0;
        m_Q(1, 1) = m_Q(0, 0);
        m_Q(0, 2) = 0.5 * (deltaT * deltaT) * processNoise;
        m_Q(1, 3) = m_Q(0, 2);
        m_Q(2, 0) = m_Q(0, 2);
        m_Q(3, 1) = m_Q(0, 2);
        m_Q(2, 2) = deltaT * processNoise;
        m_Q(3, 3) = m_Q(2, 2);
        return m_Q;
    }


    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<ConstantMotionModel> Ptr;
    typedef boost::shared_ptr<const ConstantMotionModel> ConstPtr;

    /// Return a deep copy of this motion model as a shared pointer
    virtual MotionModel::Ptr deepCopy() {
        ConstantMotionModel* copy = new ConstantMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }
};

}


#endif /* _MODELS_CONSTANT_MOTION_MODEL_H_ */

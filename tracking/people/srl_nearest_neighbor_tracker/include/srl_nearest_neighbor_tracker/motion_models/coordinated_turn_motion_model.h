/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#ifndef _MOTION_MODELS_COORDINATED_TURN_MOTION_MODEL_H_
#define _MOTION_MODELS_COORDINATED_TURN_MOTION_MODEL_H_

#include <ros/ros.h>
#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>

namespace srl_nnt {

class CoordinatedTurnMotionModel: public MotionModel
{
public:
    CoordinatedTurnMotionModel()
{
        m_A = MotionModelMatrix::Zero(DIM,DIM);
        m_Q = MotionModelMatrix::Zero(DIM,DIM);
}

    virtual const MotionModelMatrix& A (const StateVector& x, const double deltaT)  {
        m_A = MotionModelMatrix::Identity(DIM,DIM);
        ///get readable access to current state
        double omegak = x(STATE_OMEGA_IDX);
        double xk = x(STATE_X_IDX);
        double vxk = x(STATE_VX_IDX);
        double yk = x(STATE_Y_IDX);
        double vyk = x(STATE_VY_IDX);

        if (fabs(omegak) > 1e-6)
        {
            ROS_DEBUG_STREAM("Taking exact form for Coordinated turn " << fabs(omegak)
                             << " deltaT " << deltaT);

            /// calculate often needed values
            double sinOmegaT = sin(omegak *deltaT);
            double cosOmegaT = cos(omegak *deltaT);

            double omegak_2 = omegak * omegak;

            ///calculate Jacobian
            m_A(STATE_X_IDX, STATE_VX_IDX) = sinOmegaT / omegak;
            m_A(STATE_X_IDX, STATE_VY_IDX) = -1*(1- cosOmegaT)/omegak;
            m_A(STATE_VX_IDX, STATE_VX_IDX) = cosOmegaT;
            m_A(STATE_VX_IDX, STATE_VY_IDX) = -sinOmegaT;
            m_A(STATE_Y_IDX, STATE_VX_IDX) = (1- cosOmegaT)/omegak;
            m_A(STATE_Y_IDX, STATE_VY_IDX) = sinOmegaT / omegak;
            m_A(STATE_VY_IDX, STATE_VX_IDX) = sinOmegaT;
            m_A(STATE_VY_IDX, STATE_VY_IDX) = cosOmegaT;
            /// Calculate partial derivatives with respect to omega
            m_A(STATE_X_IDX, STATE_OMEGA_IDX) = (cosOmegaT * deltaT *vxk)/omegak - (sinOmegaT *vxk)/omegak_2 - (sinOmegaT * deltaT *vyk)/omegak - ((-1+cosOmegaT) *vyk)/omegak_2;
            m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = (sinOmegaT * deltaT *vxk)/omegak - ((1-cosOmegaT) *vxk)/omegak_2 + (cosOmegaT * deltaT *vyk)/omegak - (sinOmegaT *vyk)/omegak_2;
            m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = (-sinOmegaT *deltaT * vxk) - (cosOmegaT *deltaT* vyk) ;
            m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = (cosOmegaT *deltaT * vxk) - (sinOmegaT *deltaT* vyk) ;
        }
        else
        {
            ROS_DEBUG_STREAM("Taking limiting form for Coordinated turn " << fabs(omegak));
            ///calculate Jacobian limiting form
            m_A(STATE_X_IDX, STATE_VX_IDX) = deltaT;
            m_A(STATE_X_IDX, STATE_OMEGA_IDX) = -0.5*(deltaT*deltaT)*vyk;
            m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = -deltaT*vyk;
            m_A(STATE_Y_IDX, STATE_VY_IDX) = deltaT;
            m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = 0.5*(deltaT*deltaT)*vxk;
            m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = deltaT*vxk;
        }
        //ROS_WARN_STREAM("Omega = " << omegak << " and F = \n" << m_A);
        return m_A;
    }


    virtual const MotionModelMatrix& getProcessNoiseQ(const double deltaT, const double processNoise)
    {
        // get variance for turn rate omega in rad/s (default: 10 deg/s)
        double sigmaOmega = Params::get<double>("ct_turn_rate_variance", 0.17453292519);

        //Process noise according to "Modern Tracking Systems" page 210
        m_Q = MotionModelMatrix::Zero(DIM,DIM);
        /*m_Q(0, 0) = (deltaT * deltaT * deltaT * deltaT) * processNoise / 4.0;
        m_Q(1, 1) = m_Q(0, 0);
        m_Q(0, 2) = 0.5 * (deltaT * deltaT* deltaT) * processNoise;
        m_Q(1, 3) = m_Q(0, 2);
        m_Q(2, 0) = m_Q(0, 2);
        m_Q(3, 1) = m_Q(0, 2);
        m_Q(2, 2) = (deltaT*deltaT) * processNoise;
        m_Q(3, 3) = m_Q(2, 2);
        m_Q(4, 4) = (sigmaOmega * sigmaOmega) * (deltaT * deltaT);*/

        m_Q(0, 0) = (deltaT * deltaT * deltaT) * processNoise / 3.0;
        m_Q(1, 1) = m_Q(0, 0);
        m_Q(0, 2) = 0.5 * (deltaT* deltaT) * processNoise;
        m_Q(1, 3) = m_Q(0, 2);
        m_Q(2, 0) = m_Q(0, 2);
        m_Q(3, 1) = m_Q(0, 2);
        m_Q(2, 2) = deltaT * processNoise;
        m_Q(3, 3) = m_Q(2, 2);
        //m_Q(4, 4) = (sigmaOmega * sigmaOmega) * (deltaT * deltaT);
        m_Q(4, 4) = sigmaOmega * deltaT;

        return m_Q;
    }


    virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix)
    {
        MotionModelMatrix model = matrix.block<DIM,DIM>(0,0);
        return model;
    }

    virtual const MotionModelVector convertToMotionModel(const StateVector& vector)
    {
        MotionModelVector model = vector.head(DIM);
        return model;
    }

    virtual const StateMatrix convertToState(const MotionModelMatrix& matrix)
    {
        StateMatrix state = StateMatrix::Identity();
        state.block<DIM,DIM>(0,0) = matrix.block<DIM,DIM>(0,0);
        return state;
    }
    virtual const StateVector convertToState(const MotionModelVector& vector)
    {
        StateVector state = StateVector::Zero();
        state.head(DIM) = vector.head(DIM);
        return state;
    }


    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<CoordinatedTurnMotionModel> Ptr;
    typedef boost::shared_ptr<const CoordinatedTurnMotionModel> ConstPtr;

    /// Return a deep copy of this motion model as a shared pointer
    virtual MotionModel::Ptr deepCopy() {
        CoordinatedTurnMotionModel* copy = new CoordinatedTurnMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }

private:
    const static int DIM = 5;
};

}


#endif /* _MODELS_CONSTANT_MOTION_MODEL_H_ */

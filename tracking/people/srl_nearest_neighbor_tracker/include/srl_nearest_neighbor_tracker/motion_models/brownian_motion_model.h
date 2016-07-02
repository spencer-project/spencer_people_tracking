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

#ifndef _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_
#define _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_

#include <srl_nearest_neighbor_tracker/motion_models/motion_model.h>
#include <ros/ros.h>


namespace srl_nnt {

class BrownianMotionModel: public MotionModel
{
public:
    BrownianMotionModel()
{
        m_A = MotionModelMatrix::Zero(DIM,DIM);
}

    virtual const MotionModelMatrix& A (const StateVector& x, const double deltaT) {
        m_A = MotionModelMatrix::Identity(DIM,DIM);
        return m_A;
    }

    virtual const MotionModelMatrix& getProcessNoiseQ(const double deltaT, const double processNoise)
    {
        m_Q = MotionModelMatrix::Zero(DIM,DIM);
        m_Q(0, 0) = deltaT * processNoise;
        m_Q(1, 1) = m_Q(0, 0);
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
        StateMatrix state = StateMatrix::Zero();
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
    typedef boost::shared_ptr<BrownianMotionModel> Ptr;
    typedef boost::shared_ptr<const BrownianMotionModel> ConstPtr;

    /// Return a deep copy of this motion model as a shared pointer
    virtual MotionModel::Ptr deepCopy() {
        BrownianMotionModel* copy = new BrownianMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }
private:
    const static int DIM =2;

};

}


#endif /* _MOTION_MODELS_BROWNIAN_MOTION_MODEL_H_ */

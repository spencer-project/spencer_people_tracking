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

#ifndef _MOTION_MODELS_MOTION_MODEL_H_
#define _MOTION_MODELS_MOTION_MODEL_H_

#include <srl_nearest_neighbor_tracker/base/defs.h>

namespace srl_nnt
{
/// Abstract class for a generic motion model for an EKF.
class MotionModel {
public:

    typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> MotionModelMatrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> MotionModelVector;

    virtual const MotionModelMatrix& A(const StateVector& x, const double deltaT) = 0;
    virtual const MotionModelMatrix& getProcessNoiseQ(const double deltaT, const double processNoise) = 0;

    virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix)=0;
    virtual const MotionModelVector convertToMotionModel(const StateVector& vector)=0;

    virtual const StateMatrix convertToState(const MotionModelMatrix& matrix)=0;
    virtual const StateVector convertToState(const MotionModelVector& vector)=0;

    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<MotionModel> Ptr;
    typedef boost::shared_ptr<const MotionModel> ConstPtr;

    /// Return a deep copy of this filter state as a shared pointer
    virtual Ptr deepCopy() = 0;

protected:
    /// Transition matrix for motion model
    MotionModelMatrix m_A;
    MotionModelMatrix m_Q;
};

}


#endif /* _MOTION_MODELS_MOTION_MODEL_H_ */

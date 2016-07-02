/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
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

#ifndef _KALMAN_FILTER_STATE_H
#define _KALMAN_FILTER_STATE_H

#include <srl_nearest_neighbor_tracker/data/filter_state.h>


namespace srl_nnt
{
// Forward declarations (for friend declarations)
class EKF;
class LogicInitiator;
class IMMFilter;
class IMMState;


/// State and covariances of a Kalman filter.
class KalmanFilterState : public FilterState
{
protected:
    /// Mean (x) of the track state. E.g. x = {x-position, y-position, vx, vy, ax, ay}.
    StateVector m_x;

    /// Covariance matrix (C) of the track state.
    StateMatrix m_C;
private:

    /// Mean (xp) of the predicted track state.
    StateVector m_xp;

    /// Covariance (Cp) of the predicted track state.
    StateMatrix m_Cp;

public:
    // Public getters, as specified in FilterState interface
    virtual const StateVector& x()  const { return m_x; }
    virtual const StateMatrix& C()  const { return m_C; }
    virtual const StateVector& xp() const { return m_xp; }
    virtual const StateMatrix& Cp() const { return m_Cp; }


    // Public setters, as specified in FilterState interface
    virtual void setX(const StateVector& x) { m_x = x;}
    virtual void setXp(const StateVector& xp) { m_xp = xp;}
    virtual void setC(const StateMatrix& C) { m_C = C;}
    virtual void setCp(const StateMatrix& Cp) { m_Cp = Cp;}
    
    /// Typedefs for easier readability
    typedef boost::shared_ptr<KalmanFilterState> Ptr;
    typedef boost::shared_ptr<const KalmanFilterState> ConstPtr;

    /// Return a deep copy of this filter state as a shared pointer
    virtual FilterState::Ptr deepCopy() {
        KalmanFilterState* copy = new KalmanFilterState();
        *copy = *this;
        return FilterState::Ptr(copy);
    }

    // Grant write access to certain classes
    friend class EKF;
    friend class LogicInitiator;
    friend class IMMFilter;
    friend class IMMState;
};


} // end of namespace srl_nnt

#endif // _KALMAN_FILTER_STATE_H

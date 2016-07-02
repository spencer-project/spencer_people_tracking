/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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

#ifndef _FILTER_STATE_H
#define _FILTER_STATE_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{

/// Abstract class for a generic track filter's state (e.g. Kalman filter / Particle filter / IMM filter state).
class FilterState
{
public:
    /* --- Public getters. These assume the filter state and prediction can be represented as a mixture of Gaussians. --- */

    /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
    virtual const StateVector& x() const = 0;

    /// Covariance matrix (C) of the track state.
    virtual const StateMatrix& C() const = 0;

    /// Mean (xp) of the predicted track state.
    virtual const StateVector& xp() const = 0;

    /// Covariance (Cp) of the predicted track state.
    virtual const StateMatrix& Cp() const = 0;

    /// Track measurement prediction (zp).
    virtual const ObsVector&   zp() const { return m_zp; }

    /// Track measurement Jacobian (H).
    virtual const ObsStateMatrix&  H() const { return m_H; }

    /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
    virtual void setX(const StateVector& x)  = 0;

    /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
    virtual void setXp(const StateVector& xp)  = 0;

    /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
    virtual void setC(const StateMatrix& C)  = 0;

    /// Mean (x) of the track state. E.g. x = [ x-position, y-position, vx, vy ].
    virtual void setCp(const StateMatrix& Cp)  = 0;


    /* --- Other stuff --- */

    /// Typedefs for easier readability
    typedef boost::shared_ptr<FilterState> Ptr;
    typedef boost::shared_ptr<const FilterState> ConstPtr;

    /// Return a deep copy of this filter state as a shared pointer
    virtual Ptr deepCopy() = 0;

    /// Recompute the measurement prediction zp based upon the current xp and the given new Jacobian H.
    virtual void updateMeasurementPrediction(ObsStateMatrix& newH) {
        m_H = newH;
        m_zp = m_H * xp();
    }

    /// Allows streaming of a KalmanFilterState into an output stream, for debugging
    friend std::ostream& operator<< (std::ostream& stream, const FilterState::Ptr filterState) {
        stream << "State vector x:\n" << filterState->x();
        stream << "\n\nState covariance C:\n" << filterState->C();
        stream << "\n\nState prediction xp:\n" << filterState->xp(),
        stream << "\n\nState covariance prediction Cp:\n" << filterState->Cp();
        stream << "\n\nMeasurement prediction zp:\n" << filterState->zp();
        stream << "\n\nMeasurement Jacobian H:\n" << filterState->H();
        return stream;
    }

protected:
    ObsVector m_zp; /// Track measurement prediction
    ObsStateMatrix m_H; /// Track measurement Jacobian
};


} // end of namespace srl_nnt

#endif // _FILTER_STATE_H

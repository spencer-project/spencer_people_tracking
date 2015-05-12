/* Created on: Jan 03, 2015. Author: Timm Linder */
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
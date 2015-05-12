/* Created on: Jan 03, 2015. Author: Timm Linder */
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

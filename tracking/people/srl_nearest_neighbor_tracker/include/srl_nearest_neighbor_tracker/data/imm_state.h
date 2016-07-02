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

#ifndef _IMM_STATE_H_
#define _IMM_STATE_H_

#include <srl_nearest_neighbor_tracker/data/filter_state.h>
#include <srl_nearest_neighbor_tracker/data/imm_hypothesis.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#define foreach BOOST_FOREACH


namespace srl_nnt {

class EKF;
class LogicInitiator;
class IMMHypothesis;
class IMMFilter;

typedef unsigned int IMMHypothesisIndex;


class IMMState : public FilterState {
public:



    /// Typedefs for easier readability
    typedef boost::shared_ptr<IMMState> Ptr;
    typedef boost::shared_ptr<const IMMState> ConstPtr;

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


    /// Return a deep copy of this filter state as a shared pointer
    virtual FilterState::Ptr deepCopy() {
        IMMState* copy = new IMMState();
        *copy = *this;
        return FilterState::Ptr(copy);
    }

    /// Recompute the measurement prediction zp based upon the current xp and the given new Jacobian H.
    ///For IMM we have to do that for all hypotheses and call the base method as well for compatibility with interface
    virtual void updateMeasurementPrediction(ObsStateMatrix& newH) {
        //Compute mixed initial condition for all hypothesis
        FilterState::updateMeasurementPrediction(newH);
        for( int i = 0; i < m_hypotheses.size(); i++ )
        {
            m_hypotheses.at(i)->m_H = newH;
            m_hypotheses.at(i)->m_zp = m_hypotheses.at(i)->m_H * m_hypotheses.at(i)->xp();
        }
    }

    void useMixedValues()
    {
        //Compute mixed initial condition for all hypothesis
        for( int i = 0; i < m_hypotheses.size(); i++ )
        {
            m_hypotheses.at(i)->setMixedValuesForState();
        }
    }

    /// Returns the information on a specific internal hypothesis for the IMM
    IMMHypothesis::Ptr getHypothesis( int index )
    {
        return m_hypotheses.at(index);
    }

    /// Returns the current hypothesis for the IMM
    IMMHypothesis::Ptr getCurrentHypothesis()
    {
        return m_currentHypothesis;
    }

    /// Returns the index of the current hypothesis
    IMMHypothesisIndex getCurrentHypothesisIndex()
    {
        return m_currentHypothesisIdx;
    }

    IMMHypotheses getHypotheses()
    {
        return m_hypotheses;
    }


private:
    IMMHypothesis::Ptr m_currentHypothesis;
    IMMHypothesisIndex m_currentHypothesisIdx;
    IMMHypotheses m_hypotheses;

    IMMMatrix m_mixingProbabilities;

    /// Mean (x) of the track state. E.g. x = {x-position, y-position, vx, vy, ax, ay}.
    StateVector m_x;

    /// Covariance matrix (C) of the track state.
    StateMatrix m_C;

    /// Mean (xp) of the predicted track state.
    StateVector m_xp;

    /// Covariance (Cp) of the predicted track state.
    StateMatrix m_Cp;

    // Grant write access to certain classes
    friend class EKF;
    friend class LogicInitiator;
    friend class IMMHypothesis;
    friend class IMMFilter;
};

}



#endif /* _IMM_STATE_H_ */

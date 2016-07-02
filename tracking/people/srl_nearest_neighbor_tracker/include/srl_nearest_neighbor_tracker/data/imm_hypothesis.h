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

#ifndef _IMM_HYPOTHESIS_H_
#define _IMM_HYPOTHESIS_H_

#include <vector>
#include <srl_nearest_neighbor_tracker/base/defs.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>


namespace srl_nnt {

class IMMState;
class IMMFilter;

class IMMHypothesis: public KalmanFilterState {
private:

    StateVector m_xMixed;
    StateMatrix m_CMixed;

    double m_probability;
    double m_mixProbability;
    double m_likelihood;
    double m_cNormalizer;

    //FIXME ADAPTION to structure of NNT

public:
    const StateVector& xMixed()  const { return m_xMixed; }
    const StateMatrix& CMixed()  const { return m_CMixed; }


    double getProbability()
    {
        return m_probability;
    }

    double getMixedProbability()
    {
        return m_mixProbability;
    }

    void setProbability(const double p)
    {
        m_probability = p;
    }

    void setMixedProbability(const double mp)
    {
        m_mixProbability = mp;
    }

    /**
     * Swaps the mixed and regular states.  This avoid an unnecisary copy.
     */
    void setMixedValuesForState()
    {
        m_x = m_xMixed;
        m_C = m_CMixed;
    }

    typedef boost::shared_ptr<IMMHypothesis> Ptr;
    typedef boost::shared_ptr<const IMMHypothesis> ConstPtr;

    friend class IMMState;
    friend class IMMFilter;

};

typedef std::vector<IMMHypothesis::Ptr> IMMHypotheses;

}

#endif /* _IMM_HYPOTHESIS_H_ */

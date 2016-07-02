/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, Timm Linder, Fabian Girrbach, Social Robotics Lab, University of Freiburg
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
#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_MULTIVARIATE_NORMAL_DISTRIBUTION_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_MULTIVARIATE_NORMAL_DISTRIBUTION_H_


#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>


namespace srl_nnt {

/// Utility class to sample from a multi-variate normal distribution.
/// This is achieved via Eigen decomposition of the covariance matrix,
/// which is cached internally after calling setC() for faster sampling.
template<typename T>
class MultiVariateNormalDistribution
{
private:
    typedef Eigen::Matrix<T, Eigen::Dynamic,  Eigen::Dynamic> CType;
    typedef Eigen::Matrix<T, Eigen::Dynamic,  1> MuType;
    typedef Eigen::Matrix<T, Eigen::Dynamic, -1> SamplesType;
    typedef Eigen::Matrix<T, Eigen::Dynamic,  1> SampleType;

    boost::mt19937 m_randomNumberGenerator;

    mutable boost::normal_distribution<T> m_normalDistribution;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<T> > m_zeroMeanStandardMVNDistribution;
    
    Eigen::SelfAdjointEigenSolver<CType> m_eigenSolver; 
    bool m_choleskyEnabled;
    
    CType m_C, m_t;
    MuType m_mu;
    
public:
    /// typedef for easier readability
    typedef boost::shared_ptr<MultiVariateNormalDistribution> Ptr;
    typedef boost::shared_ptr<const MultiVariateNormalDistribution> ConstPtr;

    MultiVariateNormalDistribution(bool choleskyEnabled, unsigned int randomSeed):
        m_choleskyEnabled(choleskyEnabled),
        m_zeroMeanStandardMVNDistribution(m_randomNumberGenerator, m_normalDistribution)
    {
        m_randomNumberGenerator.seed(randomSeed);
    }

    void setC(const CType& C)
    {
        m_C = C;

        bool fallback = false;
        if(m_choleskyEnabled)
        {
            Eigen::LLT<CType> choleskyDecomposition(m_C);
            if(Eigen::Success == choleskyDecomposition.info())
            {
                m_t = choleskyDecomposition.matrixL();
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "Covariance matrix is not positive definite. Falling back to eigen solver instead of Cholesky decomposition!");
                fallback = true;
            }
        }

        if(!m_choleskyEnabled || fallback)
        {
            m_eigenSolver = Eigen::SelfAdjointEigenSolver<CType>(m_C);
            m_t = m_eigenSolver.eigenvectors() * m_eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        }
    }

    void setMu(const MuType& mu)
    {
        m_mu = mu;
    }

    /// Returns numSamples samples as column vectors.
    SamplesType generateSamples(unsigned int numSamples)
    {
        SamplesType allSamples(m_C.rows(), numSamples);
        SampleType currentSample(m_C.rows(), 1);

        for(int column = 0; column < numSamples; column++)
        {
            for(int row = 0; row < m_C.rows(); row++)
                currentSample(row) = m_zeroMeanStandardMVNDistribution();
            allSamples.col(column) = m_mu + m_t * currentSample;
        }
        return allSamples;
    }

}; // end class MultiVariateNormalDistribution

} // end namespace srl_nnt


#endif /* _SRL_NEAREST_NEIGHBOR_TRACKER_MULTIVARIATE_NORMAL_DISTRIBUTION_H_ */

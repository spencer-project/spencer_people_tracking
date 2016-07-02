/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Timm Linder, Social Robotics Lab, University of Freiburg
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
/*
Source: https://github.com/spaghetti-source/algorithm/blob/master/math/assignment.cc
Original license:
  These codes are published in public domain. You can use the codes for any purpose without any warranty.
  author: Takanori MAEHARA (e-mail: maehara@prefield.com / twitter: @tmaehara)

Note:
  This free implementation is about 2-5x slower compared to the original one by Jonkers, probably
  because it omits the heuristical initialization step.
*/

#ifndef _SRL_NEAREST_NEIGHBOR_TRACKER_LAP_H_
#define _SRL_NEAREST_NEIGHBOR_TRACKER_LAP_H_
#include <Eigen/Dense>

#include <vector>
#include <algorithm>

#include <srl_nearest_neighbor_tracker/base/defs.h>

namespace srl_nnt
{

/// Template class for solving the linear assignment problem
template<typename Scalar>
class LAPSolver {
public:
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> CostMatrix;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic, 1> CostVector;
  typedef Eigen::Matrix<int,Eigen::Dynamic, 1> IntVector;

private:
  typedef double value_type;
  const value_type COST_SCALE;

  value_type residue(const CostMatrix &assignmentCost, const std::vector<value_type> &v, int i, int j) {
    return value_type(assignmentCost(i, j) * COST_SCALE) - v[j];
  };

public:
    LAPSolver() : COST_SCALE(1.0) {}

    // Minimum assignment (simplified Jonker-Volgenant)
    //
    // Description:
    //   We are given a cost table of size n times m with n <= m.
    //   It finds a minimum cost assignment, i.e.,
    //     min sum_{ij} c(i,j) x(i,j)
    //     where sum_{i in [n]} x(i,j)  = 1,
    //           sum_{j in [m]} x(i,j) <= 1.
    //
    // Algorithm:
    //   Simplified version of Jonker-Volgenant algorithm,
    //   which omits a heuristical initialization step.
    //
    // Complexity:
    //   O(n^3).
    //   Much faster than the Kuhn-Munkres algorithm.
    //
    // Note:
    //   It finds minimum cost maximal matching.
    //   To find the minimum cost non-maximal matching,
    //   we add n dummy vertices to the right side.
    //
    // Verified: 
    //   SPOJ 286: Selfish City
    //
    // References:
    //   R. Jonker and A. Volgenant (1987):
    //   A shortest augmenting path algorithm for dense and sparse linear assignment problems.
    //   Computing, vol.38, no.4, pp.325-340.
    //
    //   A. Volgenant (1996): 
    //   Linear and Semi Assignment Problems: a core oriented approach.
    //   Computers and Operations Research, vol.23, pp.917-932.
    //
    // ------------------------------
    // input+output:
    //   costMatrix - cost matrix. output matrix is augmented with additional rows/columns (of BIG_COST) to make the matrix square.
    // output:
    //   rowsol     - column assigned to each row
    //   colsol     - row assigned to each column
    Scalar calculateAssignment(CostMatrix& costMatrix, IntVector& colsol, IntVector& rowsol)
    {
        int largestDim = std::max(costMatrix.cols(), costMatrix.rows());
        CostMatrix assignmentCost = CostMatrix::Constant(largestDim, largestDim, BIG_COST);
        assignmentCost.block(0,0, costMatrix.rows(), costMatrix.cols()) = costMatrix.block(0, 0, costMatrix.rows(), costMatrix.cols());

        colsol = IntVector::Zero(assignmentCost.cols());
        rowsol = IntVector::Zero(assignmentCost.rows());

        
        /*******************/

        // n = rows
        // m = cols
        const int n = assignmentCost.rows(), m = assignmentCost.cols(); // assert(n <= m);
        std::vector<value_type> v(m), dist(m);        // v: potential
        std::vector<int> matchL(n,-1), matchR(m,-1);  // matching pairs
        std::vector<int> index(m), prev(m);

        // C++11: std::iota(index.begin(), index.end(), 0);
        int currentIndex = 0;
        for(std::vector<int>::iterator indexIt = index.begin(); indexIt != index.end(); indexIt++, currentIndex++)
        {
          *indexIt = currentIndex;
        }
        
        // For each row
        for(int f = 0; f < n; ++f)
        {
          // For each column
          for(int j = 0; j < m; ++j)
          {
            dist[j] = residue(assignmentCost, v, f, j);
            prev[j] = f;
          }

          value_type w;
          int j, l;
          for(int s = 0, t = 0;;)
          {
            if(s == t)
            {
              l = s; w = dist[index[t++]]; 
              for(int k = t; k < m; ++k)
              {
                j = index[k];
                value_type h = dist[j];
                if (h <= w)
                {
                  if (h < w) { t = s; w = h; }
                  index[k] = index[t]; index[t++] = j;
                }
              }
              for(int k = s; k < t; ++k)
              {
                j = index[k];
                if (matchR[j] < 0) goto aug;
              }
            } // end if

            int q = index[s++], i = matchR[q];
            for(int k = t; k < m; ++k)
            {
              j = index[k];
              value_type h = residue(assignmentCost, v, i, j) - residue(assignmentCost, v, i, q) + w;
              if(h < dist[j])
              { 
                dist[j] = h; prev[j] = i;
                if(h == w)
                {
                  if (matchR[j] < 0) goto aug;
                  index[k] = index[t]; index[t++] = j;
                }
              }
            } // end for k
          } // end for s, t
      
      aug:
          for(int k = 0; k < l; ++k) v[index[k]] += dist[index[k]] - w;
          int i;
          do
          {
            matchR[j] = i = prev[j]; 
            std::swap(j, matchL[i]);
          }
          while (i != f);
        }

        Scalar opt = 0;
        for(int i = 0; i < n; ++i)
        {
          opt += assignmentCost(i, matchL[i]); // (i, matchL[i]) is a solution
          rowsol[i] = matchL[i];
          colsol[matchL[i]] = i;
        }

        costMatrix = assignmentCost;
        return opt;
    }

};

}

#endif //_SRL_NEAREST_NEIGHBOR_TRACKER_LAP_H_

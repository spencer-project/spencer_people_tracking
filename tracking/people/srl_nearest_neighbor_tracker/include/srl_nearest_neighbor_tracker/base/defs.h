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

/// This file contains common definitions.
#ifndef _DEFS_H
#define _DEFS_H

#define OBS_DIM 2           /// our observations consist only of x, y coordinates

// NOTE: A state dimension of 7 allows all 4 implemented motion models
// IF the Wiener Process Model is wanted STATE_DIM has to be 7
// If the highest order model is the Coordinated turn model than the State Dim could be reduced to 5
// else if for the the Constant Velocity model to 4.
#define STATE_DIM 7         /// we are tracking x, y coordinates and vx, vy velocity
#define STATE_VISUALIZATION_DIM 4         /// we visualize x, y coordinates and vx, vy velocity

#define ROS_COV_DIM 6       /// ROS pose covariances always have 6 dimensions (xyz + xyz fixed-axis rotation)
#define N_MODELS 3        	/// Number of models used for IMM filter

#define BIG_COST 100000     /// For assignment problem


#include <Eigen/Core>

namespace srl_nnt {
    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> StateMatrix;    /// square matrix of dimension STATE_DIM x STATE_DIM
    typedef Eigen::Matrix<double, STATE_DIM, 1> StateVector;            /// vector of dimension STATE_DIM

    typedef Eigen::Matrix<double, STATE_VISUALIZATION_DIM, STATE_VISUALIZATION_DIM> StateVisMatrix;    /// square matrix of dimension STATE_DIM x STATE_DIM
    typedef Eigen::Matrix<double, STATE_VISUALIZATION_DIM, 1> StateVisVector;            /// vector of dimension STATE_DIM

    typedef Eigen::Matrix<double, OBS_DIM, OBS_DIM> ObsMatrix;          /// square matrix of dimension OBS_DIM x OBS_DIM
    typedef Eigen::Matrix<double, OBS_DIM, 1> ObsVector;                /// vector of dimension OBS_DIM

    typedef Eigen::Matrix<double, OBS_DIM, STATE_DIM> ObsStateMatrix;   /// matrix of dimension OBS_DIM x STATE_DIM
    typedef Eigen::Matrix<double, STATE_DIM, OBS_DIM> StateObsMatrix;   /// matrix of dimension STATE_DIM x OBS_DIM

    typedef Eigen::Matrix<double, STATE_DIM - OBS_DIM, 1> VelocityVector; /// velocity vector at the end of a state vector

    typedef Eigen::Matrix<double, N_MODELS, N_MODELS> IMMMatrix;    /// square matrix of dimension STATE_DIM x STATE_DIM
    typedef Eigen::Matrix<double, N_MODELS, 1> IMMVector;    /// square matrix of dimension STATE_DIM x STATE_DIM

    //NOTE: Due to more efficient block operations in the motion models the order of the single state variables
    //      should not be changed. If you consider changing the order please review the motion model implementation
    const unsigned int STATE_X_IDX = 0;
    const unsigned int STATE_Y_IDX = 1;
    const unsigned int STATE_VX_IDX = 2;
    const unsigned int STATE_VY_IDX = 3;
    const unsigned int STATE_OMEGA_IDX = 4;
    const unsigned int STATE_AX_IDX = 5;
    const unsigned int STATE_AY_IDX = 6;


    /// Inverse of the chi2-cdf at 0.95 with dofs given by the array index. 
    static const double CHI2INV_95[] = { 
        -1.0,               // not available for zero dof
        3.84145882069412,   // 1
        5.99146454710798,   // 2
        7.81472790325118,   // 3
        9.48772903678115,   // 4
        11.0704976935163,   // 5
        12.591587243744     // 6
    };

    /// Inverse of the chi2-cdf at 0.99 with dofs given by the array index. 
    static const double CHI2INV_99[] = {
        -1.0,               // not available for zero dof
        6.63489660102121,   // 1
        9.21034037197618,   // 2
        11.3448667301444,   // 3
        13.2767041359876,   // 4
        15.086272469389,    // 5
        16.8118938297709    // 6
    };

    /// Inverse of the chi2-cdf at 0.999 with dofs given by the array index. 
    static const double CHI2INV_999[] = {
        -1.0,               // not available for zero dof
        10.8276,            // 1
        13.8155,            // 2
        16.2662,            // 3
        18.4668,            // 4
        20.5150,            // 5
        22.4577             // 6
    };

    /// Inverse of the chi2-cdf at 0.9999 with dofs given by the array index. 
    static const double CHI2INV_9999[] = {
        -1.0,               // not available for zero dof
        15.1367,            // 1
        18.4207,            // 2
        21.1075,            // 3
        23.5127,            // 4
        25.7448,            // 5
        27.8563             // 6
    };
}


#endif // _DEFS_H

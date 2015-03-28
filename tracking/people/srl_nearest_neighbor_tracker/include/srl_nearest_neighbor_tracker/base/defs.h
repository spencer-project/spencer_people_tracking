/// This file contains common definitions.
#ifndef _DEFS_H
#define _DEFS_H


#define OBS_DIM 2            /// our observations consist only of x, y coordinates
#define STATE_DIM 4          /// we are tracking x, y coordinates and vx, vy velocity
#define ROS_COV_DIM 6        /// ROS pose covariances always have 6 dimensions (xyz + xyz fixed-axis rotation)

#include <Eigen/Core>

namespace srl_nnt {
    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> StateMatrix;    /// square matrix of dimension STATE_DIM x STATE_DIM
    typedef Eigen::Matrix<double, STATE_DIM, 1> StateVector;            /// vector of dimension STATE_DIM

    typedef Eigen::Matrix<double, OBS_DIM, OBS_DIM> ObsMatrix;          /// square matrix of dimension OBS_DIM x OBS_DIM
    typedef Eigen::Matrix<double, OBS_DIM, 1> ObsVector;                /// vector of dimension OBS_DIM

    typedef Eigen::Matrix<double, OBS_DIM, STATE_DIM> ObsStateMatrix;   /// matrix of dimension OBS_DIM x STATE_DIM
    typedef Eigen::Matrix<double, STATE_DIM, OBS_DIM> StateObsMatrix;   /// matrix of dimension STATE_DIM x OBS_DIM

    typedef Eigen::Matrix<double, STATE_DIM - OBS_DIM, 1> VelocityVector; /// velocity vector at the end of a state vector


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
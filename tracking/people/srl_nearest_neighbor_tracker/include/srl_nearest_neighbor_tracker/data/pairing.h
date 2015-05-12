/* Created on: May 07, 2014. Author: Timm Linder */
#ifndef _PAIRING_H
#define _PAIRING_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Core>

#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{

/// A pairing is an assignment of a Track and an Observation.
struct Pairing
{
    /// The track of this pairing
    Track::Ptr track;

    /// The observation of this pairing
    Observation::Ptr observation;

    /// Flag indicating if pairing has a singular innovation covariance matrix
    bool singular;

    /// Flag indicating if pairing is validated. The Kalman filter of a track is only updated with validated pairings
    bool validated;

    /// Mahalanobis distance of a pairing
    double d;

    /// Innovation v (nu) of a pairing
    ObsVector v;

    /// Innovation covariance matrix S, inversed
    ObsMatrix Sinv;

    /// Typedefs for easier readability
    typedef boost::shared_ptr<Pairing> Ptr;
    typedef boost::shared_ptr<const Pairing> ConstPtr;
};


/// Typedef for easier readability
typedef std::vector<Pairing::Ptr> Pairings;


} // end of namespace srl_nnt


#endif // _PAIRING_H

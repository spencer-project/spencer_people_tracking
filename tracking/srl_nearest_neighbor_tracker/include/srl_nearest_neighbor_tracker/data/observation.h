#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Core>

#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{

/// Globally unique ID of an observation which is valid over the entire lifetime of the tracker
typedef unsigned int observation_id;


/// An Observation found in a sensor reading.
/// The Observation contains the 2D position z in cartesian space and its covariance R.
struct Observation
{
	/// ID of the Observation. 
	observation_id id;

	/// Creation time of the observation in seconds. Usually set from ROS header or logfile timestamp.
	double createdAt;

	/// Observation state vector z. 
	ObsVector z;

	/// Covariance R. 
	ObsMatrix R;

	/// Confidence in being a tracking target. 
	double confidence;

	/// If the observation has been matched against a track.
	bool matched;

	/// Typedefs for easier readability
	typedef boost::shared_ptr<Observation> Ptr;
	typedef boost::shared_ptr<const Observation> ConstPtr;
};


/// Typedef for easier readability
typedef std::vector<Observation::Ptr> Observations;


} // end of namespace srl_nnt


#endif // _OBSERVATION_H

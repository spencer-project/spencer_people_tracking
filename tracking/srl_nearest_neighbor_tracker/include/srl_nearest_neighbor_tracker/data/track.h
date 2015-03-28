#ifndef _TRACK_H
#define _TRACK_H

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>


#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/kalman_filter_state.h>
#include <srl_nearest_neighbor_tracker/base/defs.h>


namespace srl_nnt
{

/// Globally unique ID of a track which is valid over the entire lifetime of the tracker
typedef unsigned int track_id;


 /// A Track is the representation of an object, observed and tracked with an Extended Kalman Filter.
struct Track
{
	/// State of a track
	enum TrackStatus
	{
		NEW, MATCHED, OCCLUDED, DELETED
	};

	/// State of the track. 
	TrackStatus trackStatus;

	/// Globally unique track ID
	track_id id;

	/// Initializing or matching Observation. NULL for OCCLUDED or DELETED tracks.
	Observation::Ptr observation;

	/// Creation time of the track in seconds.
	double createdAt;

	/// Number of matches during the life-cycle of the Track. 
	unsigned int numberOfTotalMatches;

	/// Number of consecutive occlusions. 
	unsigned int numberOfConsecutiveOcclusions;

	/// Filter state (e.g. from a Kalman filter), including current and predicted track position
	FilterState::Ptr state;

	/// History of filter states for debugging.
	typedef boost::circular_buffer<FilterState::Ptr> FilterStateHistory;
	FilterStateHistory stateHistory;

	/// Typedefs for easier readability
	typedef boost::shared_ptr<Track> Ptr;
	typedef boost::shared_ptr<const Track> ConstPtr;
};


/// Typedef for easier readability
typedef std::vector<Track::Ptr> Tracks;


} // end of namespace srl_nnt


#endif // _TRACK_H

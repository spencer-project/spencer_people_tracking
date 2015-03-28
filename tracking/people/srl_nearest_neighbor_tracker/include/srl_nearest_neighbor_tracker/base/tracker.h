#ifndef _TRACKER_H
#define _TRACKER_H

#include <srl_nearest_neighbor_tracker/data/observation.h>
#include <srl_nearest_neighbor_tracker/data/track.h>


namespace srl_nnt {

/// Abstract tracker base class.
class Tracker
{
public:
    /// Destructor.
    virtual ~Tracker() {};

    /// Process a single tracking time-step using the new set of observations. Returns the currently tracked targets.
    /// The additional low-confidence observations, if any, can be used to match so-far unmatched tracks, but not for the initialization of new tracks.
    virtual const Tracks& processCycle(double currentTime, const Observations& newObservations, const Observations& additionalLowConfidenceObservations) = 0;

    /// Returns the number of the current tracking cycle, starts at 0 for the first set of observations
    virtual unsigned long int getCurrentCycleNo() = 0;
};


} // end of namespace srl_nnt


#endif // _TRACKER_H
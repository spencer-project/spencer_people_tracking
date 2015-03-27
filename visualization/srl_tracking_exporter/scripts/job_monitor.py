#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Waits for one or multiple nodes to finish processing (which is reported by calling a service method),
# then this node kills itself. This is useful in a ROS launch file to quit the entire launch script if
# required processing is finished (by setting required=true on node_monitor).
#
# Parameters:
#  _count (int):    Number of jobs to wait for.

import os, sys, time
import roslib, rospy, tf; roslib.load_manifest('srl_tracking_exporter')
from srl_tracking_exporter.srv import JobFinished

### Callback when a job is finished ###
def jobFinished(request):
    global numFinishedJobs
    numFinishedJobs += 1
    rospy.loginfo("Job has finished: %s, %d job(s) remaining!" % (request.job_name, numTotalJobs - numFinishedJobs))
    
### Main method ###
def main(argv=None):
    rospy.init_node("job_monitor")

    global numTotalJobs
    numTotalJobs = rospy.get_param('~count', 0)
    if 0 == numTotalJobs:
        rospy.logerror("Job count is 0, not monitoring any jobs, terminating!")

    else:
        rospy.loginfo("Will run until %d jobs have been completed." % numTotalJobs)

        jobFinishedService = rospy.Service('job_monitor', JobFinished, jobFinished)
        while numFinishedJobs < numTotalJobs and not rospy.is_shutdown():
            time.sleep(0.1)

        if numFinishedJobs < numTotalJobs:
            rospy.logwarn("Job monitor has been requested to terminate while %d job(s) were still active." % (numTotalJobs - numFinishedJobs))
        else:
            rospy.loginfo("All jobs have finished! Terminating job monitor.")
        sys.exit(0)

### Globals ###
numTotalJobs = 0
numFinishedJobs = 0
if __name__ == "__main__":
    main()
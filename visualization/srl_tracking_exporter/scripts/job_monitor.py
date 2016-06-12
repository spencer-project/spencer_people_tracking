#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Waits for one or multiple nodes to finish processing (which is reported by calling a service method),
then this node kills itself. This is useful in a ROS launch file to quit the entire launch script if
required processing is finished (by setting required=true on node_monitor).

Parameters:
 _count (int):    Number of jobs to wait for.
"""

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
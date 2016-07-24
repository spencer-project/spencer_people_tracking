#!/usr/bin/env python

# Software License Agreement (BSD License)
# 
# Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Interpolates the tick values contained in AdditionalOdometryData messages of a bagfile,
# to achieve a higher odometry / TF framerate. Only makes sense if used in combination with
# reconstruct_odometry.py at runtime.
#
# !!! Assumes that the bagfile has been synced before using postprocess_bagfile.py !!!
#

import rospy
import sys, rosbag, os.path, copy, math
import subprocess, yaml

COLOR_ERROR = '\033[91m'

if len(sys.argv) < 3:
    sys.stderr.write(COLOR_ERROR + 'Missing arguments, syntax is INPUT_FILE DESIRED_ODOM_FREQUENCY !\n')
    sys.exit(1)

desiredFrequency = float(sys.argv[2])
infilename = sys.argv[1]
outfilename = infilename + ".interpolated"

if os.path.isfile(outfilename):
    print COLOR_ERROR +  'Output file ' + outfilename + ' already exists, cannot proceed!!'
    sys.exit(2) 

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', infilename], stdout=subprocess.PIPE).communicate()[0])

topicName = "additional_odom_data"
additionalOdomMsgs = 0

msg_count = 0
topics = set()
for topicInfo in info_dict["topics"] : 
    msg_count += topicInfo["messages"]
    topic = topicInfo["topic"]
    topics.add(topic)
    if topic == topicName:
        additionalOdomMsgs += topicInfo["messages"]
    print("  %s (%d messages)" % (topic, topicInfo["messages"]))

if not topicName in topics:
    print COLOR_ERROR +  'Bag file does not contain any messages at additional_odom_data topic!'
    sys.exit(2) 

print '\nInterpolating encoder ticks in AdditionalOdometryData messages of bagfile ' + infilename + ' to ' + str(desiredFrequency) + ' Hz!'

newAdditionalOdomMsgs = 0
msg_index = 0
outbag = rosbag.Bag(outfilename, 'w', chunk_threshold = 50*1024*1024)
try:
    lastAdditionalOdomMsg = None
    for topic, msg, timestamp in rosbag.Bag(infilename).read_messages():
        if topic == topicName:
            if lastAdditionalOdomMsg is not None:
                actual_dt = (msg.header.stamp - lastAdditionalOdomMsg.header.stamp).to_sec()
                if actual_dt > 1.0 / desiredFrequency:
                    desired_dt = 1.0 / desiredFrequency
                    requiredAdditionalMessages = int(math.ceil(actual_dt / desired_dt))
                    for i in xrange(1, requiredAdditionalMessages):
                        progress = 1.0/requiredAdditionalMessages*i
                        additionalMsg = copy.deepcopy(lastAdditionalOdomMsg)
                        additionalMsg.header.stamp  = lastAdditionalOdomMsg.header.stamp + (msg.header.stamp - lastAdditionalOdomMsg.header.stamp)*progress
                        if additionalMsg.header.stamp >= msg.header.stamp:
                            continue
                        additionalMsg.totalDistance = lastAdditionalOdomMsg.totalDistance + (msg.totalDistance - lastAdditionalOdomMsg.totalDistance)*progress
                        additionalMsg.theta = lastAdditionalOdomMsg.theta + (msg.theta - lastAdditionalOdomMsg.theta)*progress
                        additionalMsg.hardwareTimestamp = lastAdditionalOdomMsg.hardwareTimestamp + (msg.hardwareTimestamp - lastAdditionalOdomMsg.hardwareTimestamp)*progress
                        additionalMsg.ticksLeft = lastAdditionalOdomMsg.ticksLeft + (msg.ticksLeft - lastAdditionalOdomMsg.ticksLeft)*progress
                        additionalMsg.ticksRight = lastAdditionalOdomMsg.ticksRight + (msg.ticksRight - lastAdditionalOdomMsg.ticksRight)*progress
                        additionalMsg.calibOverallMultiplier = lastAdditionalOdomMsg.calibOverallMultiplier + (msg.calibOverallMultiplier - lastAdditionalOdomMsg.calibOverallMultiplier)*progress
                        additionalMsg.calibLeftEncMultiplier = lastAdditionalOdomMsg.calibLeftEncMultiplier + (msg.calibLeftEncMultiplier - lastAdditionalOdomMsg.calibLeftEncMultiplier)*progress

                        outbag.write(topic, additionalMsg, additionalMsg.header.stamp)
                        newAdditionalOdomMsgs += 1

            lastAdditionalOdomMsg = msg


        # Write all input messages
        outbag.write(topic, msg, timestamp)

        # Show status info
        msg_index+=1
        if msg_index % (msg_count / 100) == 0:
            percent = int(100.0 * msg_index / msg_count + 0.5)
            progressBar = u"[" + u"\u2588" * percent + u"\u2591" * (100-percent) + "]"
            sys.stdout.write("\r%s %d %% completed" % (progressBar, percent))
            sys.stdout.flush()

finally:
    outbag.close()

print("\rOdometry interpolation complete! Bag file now has %d AdditionalOdometryData msgs (was %d)!%s\n" % (additionalOdomMsgs + newAdditionalOdomMsgs, additionalOdomMsgs, " " * 120))

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

# Shifts all timestamps in a bagfile by the given amount of seconds.
# !!! Assumes that the bagfile has been synced before using postprocess_bagfile.py !!!
#

import rospy
import sys, rosbag, os.path
import subprocess, yaml

COLOR_ERROR = '\033[91m'

if len(sys.argv) < 3:
    sys.stderr.write(COLOR_ERROR + 'Missing arguments, syntax is INPUT_FILE DELAY_SECONDS !\n')
    sys.exit(1)

delaySeconds = float(sys.argv[2])
infilename = sys.argv[1]
outfilename = infilename + ".shifted"

if os.path.isfile(outfilename):
    print COLOR_ERROR +  'Output file ' + outfilename + ' already exists, cannot proceed!!'
    sys.exit(2) 

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', infilename], stdout=subprocess.PIPE).communicate()[0])

msg_count = 0
for topicInfo in info_dict["topics"] : 
    msg_count += topicInfo["messages"]
    topic = topicInfo["topic"]
    print("  %s (%d messages)" % (topic, topicInfo["messages"]))


print '\nShifting timestamps in bagfile ' + infilename + ' by ' + str(delaySeconds) + ' seconds!'

msg_index = 0
outbag = rosbag.Bag(outfilename, 'w', chunk_threshold = 50*1024*1024)
try:
    for topic, msg, timestamp in rosbag.Bag(infilename).read_messages():
        # Fix header timestamps
        if msg._has_header :
            msg.header.stamp += rospy.Duration(delaySeconds)

        # Fix TF transforms
        if topic == "/tf":
            for i in range(0, len(msg.transforms)):
                msg.transforms[i].header.stamp += rospy.Duration(delaySeconds)

        # Write message
        if not topic == "/rosout":
            outbag.write(topic, msg, timestamp + rospy.Duration(delaySeconds))

        # Show status info
        msg_index+=1
        if msg_index % (msg_count / 100) == 0:
            percent = int(100.0 * msg_index / msg_count + 0.5)
            progressBar = u"[" + u"\u2588" * percent + u"\u2591" * (100-percent) + "]"
            sys.stdout.write("\r%s %d %% completed" % (progressBar, percent))
            sys.stdout.flush()

finally:
    outbag.close()

print("\rTime-shifting process complete!%s\n" % (" " * 120))

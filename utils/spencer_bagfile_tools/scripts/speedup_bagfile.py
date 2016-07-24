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

# Script to speed up the timings in a recorded bagfile by a given factor.
# !!! Assumes that the bagfile has been synced before using sync_bagfile.py !!!

import roslib; roslib.load_manifest('rosbag')
import sys, rosbag, os.path
import subprocess, yaml

if len(sys.argv) < 4:
    sys.stderr.write('Missing arguments, syntax is RATE INPUT_FILE OUTPUT_FILE !\n')
    sys.exit(1)

rate = float(sys.argv[1])
infilename = sys.argv[2]
outfilename = sys.argv[3]

if os.path.isfile(outfilename):
    print 'Output file ' + outfilename + ' already exists, cannot proceed!!'
    sys.exit(2) 

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', infilename], stdout=subprocess.PIPE).communicate()[0])

msg_count = 0
for topic in info_dict["topics"] : 
    msg_count += topic["messages"]

print '\nSpeeding up bag file ' + infilename + ' consisting of ' + str(msg_count) + ' messages by a factor of ' + str(rate)

gotFirstTimestamp = False
msg_index = 0
outbag = rosbag.Bag(outfilename, 'w')
try:
    for topic, msg, timestamp in rosbag.Bag(infilename).read_messages():
        if not gotFirstTimestamp :
            firstTimestamp = timestamp
            gotFirstTimestamp = True

        # Calculate corrected timestamp
        correctedTimestamp = firstTimestamp + (timestamp - firstTimestamp)/rate

        # Fix header timestamps
        if msg._has_header :
            msg.header.stamp = correctedTimestamp

        # Fix TF transforms
        if topic == "/tf":
            for i in range(0, len(msg.transforms)):
                msg.transforms[i].header.stamp = correctedTimestamp

        # Write message
        if not topic == "/rosout":
            outbag.write(topic, msg, correctedTimestamp)

        # Show status info
        msg_index+=1
        if msg_index % (msg_count / 10) == 0:
            print str(int(100.0 * msg_index / msg_count + 0.5)) + '% completed'

finally:
    outbag.close()

print 'Bagfile processing complete!\n'

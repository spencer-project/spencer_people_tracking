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

# Replaces TF frames in a bagfile using the provided dictionary of frame remappings.
#

import rospy
import sys, rosbag, os.path
import subprocess, yaml

rospy.init_node('replace_frames', anonymous=True)

COLOR_ERROR = '\033[91m'

if len(sys.argv) < 2:
    sys.stderr.write(COLOR_ERROR + "Need to specify INPUT_FILE and replacements:={ old_frame123: new_frame456, ...} !\n")
    sys.exit(1)

infilename = sys.argv[1]
outfilename = infilename + ".processed"

if os.path.isfile(outfilename):
    print COLOR_ERROR +  'Output file ' + outfilename + ' already exists, cannot proceed!!'
    sys.exit(2) 

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', infilename], stdout=subprocess.PIPE).communicate()[0])

msg_count = 0
for topicInfo in info_dict["topics"] : 
    msg_count += topicInfo["messages"]
    topic = topicInfo["topic"]
    print("  %s (%d messages)" % (topic, topicInfo["messages"]))

replacements = rospy.get_param("~replacements", dict())
if not replacements:
    print COLOR_ERROR + "No replacements have been specified!"
    sys.exit(3)

print 'Replacing TF frames in bagfile ' + infilename + ': ' + str(replacements) + "\n"

msg_index = 0
outbag = rosbag.Bag(outfilename, 'w', chunk_threshold = 50*1024*1024)
replaced = []
try:
    for topic, msg, timestamp in rosbag.Bag(infilename).read_messages():
        # Fix header timestamps
        if msg._has_header :
            oldFrame = msg.header.frame_id
            if oldFrame in replacements.keys():
                newFrame = replacements[oldFrame].strip()
                msg.header.frame_id = newFrame

                if not oldFrame in replaced:
                    replaced.append(oldFrame)
                    sys.stderr.write("Found in bag: " + oldFrame + " --> " + newFrame + "\n")

        # Write message
        if not topic == "/rosout":
            outbag.write(topic, msg, timestamp)

        # Show status info
        msg_index+=1
        if msg_index % (msg_count / 10) == 0:
            percent = msg_index / float(msg_count) * 100.0 + 0.5
            sys.stdout.write("%d %%..." % percent)
            sys.stdout.flush()

finally:
    outbag.close()

print("\nTF frame replacement complete!\n")

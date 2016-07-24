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


# This script does the following with a given bagfile:
# 
# 1.) Replaces message timestamps in a bag with timestamps from message headers
#
# See http://code.ros.org/lurker/message/20100901.154936.67720d20.en.html#ros-users
# and http://www.ros.org/wiki/rosbag/Cookbook
#
# 2.) It applies a preset topic renaming scheme, that also takes the bag file's filename
# into account (e.g. readings from rear RGB-D sensors will be prefixed with rear_rgbd).
#
# 3.) It renames TF frames to make them unique, and adhere to the frames defined in
# https://svncvpr.informatik.tu-muenchen.de/redmine/projects/spencer_wiki/wiki/Spencer_ros_frames
#
# 4.) It removes the leading / from all topic names, such that topics can more easily
# be remapped into a sub-namespace.
#
# (C)2014 Timm Linder, Social Robotics Lab, Uni Freiburg

import rospy, sys, rosbag, os.path, re, subprocess, yaml

def renameTopic(topic, inputFilename):
    renamedTopic = topic

    # Rename front/rear RGB-D
    pattern = re.compile("/rgbd(0|1)(/.*)")
    match = pattern.match(topic)
    if match:
        direction = "rear" if "rear_" in inputFilename else "front"
        position = "top" if match.group(1) == "0" else "bottom"
        if direction == "rear":  # swap position on rear
            position = "bottom" if position == "top" else "top"

        renamedTopic = "/rgbd_%s_%s" % (direction, position)
        renamedTopic += match.group(2)

        # To avoid conflicts with image_proc/debayer and depth_processing
        renamedTopic = renamedTopic.replace("rgb/image_raw/compressed", "rgb_recorded/image_color/compressed")
        renamedTopic = renamedTopic.replace("depth/image_raw/compressedDepth", "depth_recorded/image_raw/compressedDepth")

    # Rename Kinect2 
    if "kinect2" in inputFilename:
        renamedTopic = "/kinect2" + topic.replace("srl_dataset_publisher/floor", "initial_floor_estimate")

    # Rename DSLR
    if "dslr" in inputFilename:
        renamedTopic = "/dslr/" + renamedTopic

    # Rename lasers
    pattern = re.compile("/(.*)_laser(/.*)")
    match = pattern.match(topic)
    if match:
        renamedTopic = "/laser_" + match.group(1) + match.group(2)

    # Prepend /spencer/sensors prefix
    # NOTE: It is apparently not possible to push down topics into a sub-namespace with rosbag play or rqt_bag.
    if renamedTopic[0] == "/":
        renamedTopic = renamedTopic[1:]

    return renamedTopic


def renameFrame(frame, inputFilename):
    renamedFrame = frame

    pattern = re.compile("/rgbd(0|1)_rgb_optical_frame")
    match = pattern.match(frame)
    if match:
        direction = "rear" if "rear_" in inputFilename else "front"
        position = "top" if match.group(1) == "0" else "bottom"
        if direction == "rear":  # swap position on rear
            position = "bottom" if position == "top" else "top"

        renamedFrame = "rgbd_%s_%s_rgb_optical_frame" % (direction, position)

    if "laser" in inputFilename:
        renamedFrame = renamedFrame.replace("front_laser", "laser_front_link")
        renamedFrame = renamedFrame.replace("rear_laser",  "laser_rear_link")

    if "dslr" in inputFilename:
        renamedFrame = renamedFrame.replace("base_link", "dslr_optical_frame")

    return renamedFrame



COLOR_ERROR = '\033[91m'
if len(sys.argv) < 2:
    sys.stderr.write(COLOR_ERROR + 'Must specify name of bag file!\n')
    sys.exit(1)

inputFilename = sys.argv[1]
outputFilename = inputFilename + '.processed'
if os.path.isfile(outputFilename):
    sys.stderr.write(COLOR_ERROR + 'Synced file ' + outputFilename + ' already exists, skipping synchronization!\n')
    sys.exit(2) 

print('Retrieving bag file info... this may take a while...')
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', sys.argv[1]], stdout=subprocess.PIPE).communicate()[0])
print('Post-processing bag file ' + inputFilename)

msg_count = 0
for topicInfo in info_dict["topics"] : 
    msg_count += topicInfo["messages"]
    topic = topicInfo["topic"]
    renamedTopic = renameTopic(topic, inputFilename)

    print("  %s (%d messages) %s" % (topic, topicInfo["messages"], ("--> " + renamedTopic) if renamedTopic != topic else ""))

print("")
aborted = False
msg_index = 0
outbag = rosbag.Bag(outputFilename, 'w', chunk_threshold = 50*1024*1024)
try:
    for topic, msg, t in rosbag.Bag(inputFilename).read_messages():
        msg_index += 1
        if msg_index % (msg_count / 100) == 0:
            percent = int(100.0 * msg_index / msg_count + 0.5)
            progressBar = u"[" + u"\u2588" * percent + u"\u2591" * (100-percent) + "]"
            sys.stdout.write("\r%s %d %% completed" % (progressBar, percent))
            sys.stdout.flush()

        if msg._has_header :
            msg.header.frame_id = renameFrame(msg.header.frame_id, inputFilename)

        if topic == "/tf" and len(msg.transforms) > 0:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        
        elif not topic == "/rosout":
            outbag.write(renameTopic(topic, inputFilename), msg, msg.header.stamp if msg._has_header else t)

except KeyboardInterrupt:
    aborted = True
finally:
    outbag.close()


if aborted:
    sys.stderr.write('\n\n' + COLOR_ERROR + "Aborted by keyboard interrupt\n")
else:
    print("\rPost-processing complete!%s\n" % (" " * 120))

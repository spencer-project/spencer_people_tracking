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

# Creates thumbnails of a given image topic at regular intervals
# in the current folder. The filename corresponds to the timestamp.
#

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rospy, numpy
import sys, rosbag, os.path
import subprocess, yaml

COLOR_ERROR = '\033[91m'

if len(sys.argv) < 4:
    sys.stderr.write(COLOR_ERROR + 'Missing arguments, syntax is INPUT_FILE TOPIC INTERVAL_SECS [JUST_DISPLAY]!\n')
    sys.exit(1)

topicName = sys.argv[2]
infilename = sys.argv[1]
interval = float(sys.argv[3])

if len(sys.argv) > 4:
    justShow = bool(sys.argv[4])
else:
    justShow = False

print "Preparsing bag file..."
info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', infilename], stdout=subprocess.PIPE).communicate()[0])

if info_dict is None:
    sys.stderr.write(COLOR_ERROR + 'Specified bag file does not exist or could not be read: %s\n' % infilename)
    sys.exit(1)

msg_count = 0
topics = []
for topicInfo in info_dict["topics"] : 
    msg_count += topicInfo["messages"]
    topics.append(topicInfo["topic"])

if not topicName in topics:
    sys.stderr.write(COLOR_ERROR + "Topic %s doesn't exist in bagfile! Existing topics are:\n%s\n" % (topicName, str(topics)))
    sys.exit(1)

msg_index = 0

bridge = CvBridge()
lastThumbnailAt = -1000000

def processImage(cv_image):
    global lastThumbnailAt
    lastThumbnailAt = msg.header.stamp.to_sec()

    cv2.putText(cv_image, "%.3f" % msg.header.stamp.to_sec(), (1, 17), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, cv2.CV_AA )
    cv2.putText(cv_image, "%.3f" % msg.header.stamp.to_sec(), (0, 16), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1, cv2.CV_AA )
    
    if not justShow:
        filename = "%.3f.jpg" % msg.header.stamp.to_sec()
        cv2.imwrite(filename, cv_image)

    cv2.imshow("Bagfile preview: %s" % topicName, cv_image)
    cv2.waitKey(1)

print "Generating thumbnails in current working directory..."
for topic, msg, timestamp in rosbag.Bag(infilename).read_messages():
    # Check for correct topic
    if topic == topicName:
        if msg.__class__.__name__ == "_sensor_msgs__Image":
            if msg.header.stamp.to_sec() - lastThumbnailAt > interval:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                processImage(cv_image)
        elif msg.__class__.__name__ == "_sensor_msgs__CompressedImage":
            if msg.header.stamp.to_sec() - lastThumbnailAt > interval:
                cv_image = cv2.imdecode(numpy.frombuffer(msg.data, dtype='uint8'), cv2.CV_LOAD_IMAGE_COLOR)
                processImage(cv_image)       
        else:
            sys.stderr.write(COLOR_ERROR + "Topic %s is of wrong type (%s)! Expected sensor_msgs/Image!\n" % (topicName, msg.__class__.__name__.replace('__', '/')[1:]))
            sys.exit(1)

    # Show status info
    msg_index+=1
    if msg_index % (msg_count / 100) == 0:
        percent = int(100.0 * msg_index / msg_count + 0.5)
        progressBar = u"[" + u"\u2588" * percent + u"\u2591" * (100-percent) + "]"
        sys.stderr.write("\r%s %d %% completed" % (progressBar, percent))
        sys.stderr.flush()


sys.stderr.write("\Generation of thumbnails complete!%s\n" % (" " * 120))

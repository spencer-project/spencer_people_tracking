#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2014, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## Based upon tf_remap from the ROS tf package
## Filters out specific tf transforms, all others are relayed as-is
## Listens on /tf_old, publishes /tf.
##
## Run like this:
## rosrun kindercar_data_capture tf_filter _exclude:='[{parentFrame: '/odom', childFrame: '/base_link'}, ...]'

import roslib; roslib.load_manifest('tf')
import rospy
from tf.msg import tfMessage
from sets import Set


class TfFilter:
    def buildHash(self, parentFrame, childFrame) :
        return "parentFrame: " + parentFrame + ", childFrame: " + childFrame

    def __init__(self):
        excludeArgument = rospy.get_param('~exclude', [])
        self.excludes = Set()
        
        for exclude in excludeArgument:
            parentFrame = exclude["parentFrame"]
            childFrame = exclude["childFrame"]
            
            hash = self.buildHash(parentFrame, childFrame)
            self.excludes.add(hash)

        self.publisher = rospy.Publisher('/tf', tfMessage, queue_size=20)
        self.subscriber = rospy.Subscriber("/tf_old", tfMessage, self.callback)
    
        print("Re-publishing all transforms from " + self.subscriber.name + " to " + self.publisher.name 
            + " except for the following transforms: ")

        for exclude in self.excludes:
            print "  " + exclude

    def callback(self, tf_msg):
        for transform in tf_msg.transforms:
            hash = self.buildHash(transform.header.frame_id, transform.child_frame_id)
            #print("hash: " + hash)
            if hash in self.excludes:
                #print("Filtering out TF -- " + repr(transform))
                tf_msg.transforms.remove(transform);
                
        self.publisher.publish(tf_msg)

if __name__ == '__main__':
    rospy.init_node('tf_filter')
    tff = TfFilter()
    rospy.spin()

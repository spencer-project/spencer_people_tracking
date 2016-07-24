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

import rospy, tf
from std_msgs.msg import Float32

rospy.init_node("plot_tf")
listener = tf.TransformListener()

pub = rospy.Publisher("tf_heartbeat", Float32, queue_size=5)
latencyPub = rospy.Publisher("tf_latency_ms", Float32, queue_size=5)

currentTime = rospy.Time.now()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
    if rospy.Time.now() > currentTime:
        currentTime = rospy.Time.now()
        try:
            lastTransformAt = listener.getLatestCommonTime('map', 'odom')
        except tf.Exception:
            lastTransformAt = rospy.Time(0)
            pass

        latency = (currentTime - lastTransformAt).to_sec()

        if latency < 0.3:
            value = 300
        else:
            value = 0

        msg = Float32()
        msg.data = value
        pub.publish(msg)

        if latency < 10:
            latencyMsg = Float32()
            latencyMsg.data = latency * 1000.0
            latencyPub.publish(latencyMsg)






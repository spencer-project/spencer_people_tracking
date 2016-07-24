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

import rospy
from spencer_control_msgs.msg import SystemStatus
from geometry_msgs.msg import Twist

def twistCallback(twist):
    global cachedTwist
    cachedTwist = twist

def systemStatusCallback(systemStatus):
    global cachedTwist
    if cachedTwist is not None:
        stamp = systemStatus.header.stamp.to_sec()
        ticks_left = systemStatus.wheel_encoder_left_ticks
        ticks_right = systemStatus.wheel_encoder_right_ticks

        v = cachedTwist.linear.x
        omega = cachedTwist.angular.z

        csv.write("%f\t%f\t%f\t%f\t%f\n" % (stamp, v, omega, ticks_left, ticks_right) )

        cachedTwist = None


cachedTwist = None

rospy.init_node("dump_encoder_values")
rospy.loginfo("Dumping encoder values into CSV!")

csv = open("encoder_values.csv", "w")
csv.write("stamp\tv\tomega\tticks_left\tticks_right\n")

twistSubscriber = rospy.Subscriber("/spencer/control/drive_velocity", Twist, twistCallback)
systemStatusSubscriber = rospy.Subscriber("/spencer/control/system_status", SystemStatus, systemStatusCallback)

rospy.spin()
csv.close()
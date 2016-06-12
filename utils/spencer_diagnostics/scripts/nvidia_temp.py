#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

##\author Kevin Watts
##\brief Publishes diagnostic data on temperature and usage for a Quadro 600 GPU

from __future__ import with_statement, division

PKG = 'pr2_computer_monitor'
import roslib

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from pr2_msgs.msg import GPUStatus

import spencer_diagnostics

class NVidiaTempMonitor(object):
    def __init__(self):
        self._pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=3)
        self._gpu_pub = rospy.Publisher('gpu_status', GPUStatus, queue_size=3)

    def pub_status(self):
        gpu_stat = GPUStatus()
        stat = DiagnosticStatus()
        try:
            card_out = spencer_diagnostics.get_gpu_status()
            gpu_stat = spencer_diagnostics.parse_smi_output(card_out)
            stat = spencer_diagnostics.gpu_status_to_diag(gpu_stat)
        except Exception, e:
            import traceback
            rospy.logerr('Unable to process nVidia GPU data')
            rospy.logerr(traceback.format_exc())

        gpu_stat.header.stamp = rospy.get_rostime()

        array = DiagnosticArray()
        array.header.stamp = rospy.get_rostime()
        
        array.status = [ stat ]

        self._pub.publish(array)
        self._gpu_pub.publish(gpu_stat)

if __name__ == '__main__':
    rospy.init_node('nvidia_temp_monitor')
    
    monitor = NVidiaTempMonitor()
    my_rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        monitor.pub_status()
        my_rate.sleep()

                        

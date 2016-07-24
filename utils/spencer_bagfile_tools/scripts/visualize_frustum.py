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

# Publishes visualization markers to display a view frustum of a camera sensor using lines.

import rospy, math, numpy, tf, time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def toRad(deg):
    return deg * math.pi / 180.0

def visualizeFrustum():
    rospy.init_node("visualize_frustum")

    markerPublisher = rospy.Publisher("/spencer_bagfile_tools/visualized_frustums", Marker, queue_size=1)

    fov_h = rospy.get_param("~fov_h", 60.0)
    fov_v = rospy.get_param("~fov_v", 40.0)
    distance = rospy.get_param("~distance", 5.0)

    marker = Marker()
    marker.header.frame_id = rospy.get_param("~frame", "base_link")

    marker.type = Marker.LINE_LIST
    marker.ns = rospy.get_param("~ns", "")
    marker.id = rospy.get_param("~id", 0)
    marker.color.r = rospy.get_param("~r", 1.)
    marker.color.g = rospy.get_param("~g", 1.)
    marker.color.b = rospy.get_param("~b", 1.)
    marker.color.a = rospy.get_param("~a", 1.)
    marker.scale.x = rospy.get_param("~line_width", 1.) * 0.005

    halfFrustumWidth  = math.sin(toRad(fov_h) / 2.0) * distance
    halfFrustumHeight = math.sin(toRad(fov_v) / 2.0) * distance

    origin = Point(x=0, y=0, z=0)
    topLeft     = Point(x=distance, y=+halfFrustumWidth, z=+halfFrustumHeight)
    topRight    = Point(x=distance, y=-halfFrustumWidth, z=+halfFrustumHeight)
    bottomLeft  = Point(x=distance, y=+halfFrustumWidth, z=-halfFrustumHeight)
    bottomRight = Point(x=distance, y=-halfFrustumWidth, z=-halfFrustumHeight)


    marker.points.append(origin);  marker.points.append(topLeft)

    marker.points.append(origin);  marker.points.append(topRight)

    marker.points.append(origin);  marker.points.append(bottomLeft)

    marker.points.append(origin);  marker.points.append(bottomRight)

    marker.points.append(topLeft); marker.points.append(topRight)

    marker.points.append(topRight); marker.points.append(bottomRight)

    marker.points.append(bottomRight); marker.points.append(bottomLeft)

    marker.points.append(bottomLeft); marker.points.append(topLeft)


    rospy.loginfo("Visualizing %d deg x %d deg x %d m frustum for TF frame '%s' with marker ID %d in marker namespace '%s'" % (fov_h, fov_v, distance, marker.header.frame_id, marker.id, marker.ns) )

    
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        markerPublisher.publish(marker)

        time.sleep(1.0 / 30)

if __name__ == '__main__':  
    visualizeFrustum()

        


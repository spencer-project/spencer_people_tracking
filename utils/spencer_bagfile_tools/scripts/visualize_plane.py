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

# Publishes a visualization marker to display a solid ground plane.

import rospy, math, numpy, tf, time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def visualizePlane():
    rospy.init_node("visualize_frustum")

    markerPublisher = rospy.Publisher("/spencer_bagfile_tools/visualized_plane", Marker, queue_size=1)

    marker = Marker()
    marker.header.frame_id = rospy.get_param("~frame", "odom")

    marker.type = Marker.MESH_RESOURCE
    marker.ns = rospy.get_param("~ns", "")
    marker.id = rospy.get_param("~id", 0)
    marker.color.r = rospy.get_param("~r", 0.5)
    marker.color.g = rospy.get_param("~g", 0.5)
    marker.color.b = rospy.get_param("~b", 0.5)
    marker.color.a = rospy.get_param("~a", 1.)
    marker.scale.x = marker.scale.y = 0.5 * rospy.get_param("~size", 100)

    marker.pose.position.x = rospy.get_param("~x", 0)
    marker.pose.position.y = rospy.get_param("~y", 0)
    marker.pose.position.z = rospy.get_param("~z", 0)

    marker.mesh_resource = "package://kindercar_meshes/meshes/simple_plane.dae"
    marker.mesh_use_embedded_materials = False

    rospy.loginfo("Visualizing plane in TF frame %s" % (marker.header.frame_id) )

    try:
        while not rospy.is_shutdown():
            marker.header.stamp = rospy.Time.now()
            markerPublisher.publish(marker)
            time.sleep(1.0 / 30)
            
    except rospy.exceptions.ROSInterruptException:
        pass

if __name__ == '__main__':  
    visualizePlane()

        


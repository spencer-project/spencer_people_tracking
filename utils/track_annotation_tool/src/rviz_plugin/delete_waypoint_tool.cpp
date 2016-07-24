/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>
#include <std_msgs/Header.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>
#include "delete_waypoint_tool.h"

namespace track_annotation_tool
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
DeleteWaypointTool::DeleteWaypointTool()
{
  shortcut_key_ = 'd';
}

DeleteWaypointTool::~DeleteWaypointTool()
{
}

void DeleteWaypointTool::onInitialize()
{
  pub_ = nh_.advertise<std_msgs::Header>( "track_annotation_tool/delete_active_waypoint", 1 );
  setName("Delete active waypoint (D)");
}

int DeleteWaypointTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  return Render | Finished;
}

// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void DeleteWaypointTool::activate()
{
  // Notify listeners of deleted waypoint via ROS
  deleteLastWaypoint();

  // Show status message
  setStatus("Active waypoint has been deleted");

  // Switch back to "place waypoint" tool after this tool is deactivated (by mouse move event in viewport)
  rviz::ToolManager* toolManager = context_->getToolManager();

  for(int i = 0; i < toolManager->numTools(); i++) {
    rviz::Tool* tool = toolManager->getTool(i);
    if(tool->getClassId().toStdString() == "track_annotation_tool/PlaceWaypoint") {
      toolManager->setDefaultTool(tool);
      break;
    }
  }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void DeleteWaypointTool::deactivate()
{
}

void DeleteWaypointTool::deleteLastWaypoint()
{
  static unsigned int sequenceCounter = 0;
  std_msgs::Header deleteMsg;
  deleteMsg.seq = sequenceCounter++;
  deleteMsg.stamp = ros::Time::now();
  pub_.publish( deleteMsg );
}

} // end namespace track_annotation_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(track_annotation_tool::DeleteWaypointTool, rviz::Tool)
// END_TUTORIAL

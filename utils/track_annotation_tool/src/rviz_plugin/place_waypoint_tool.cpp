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

#include <sstream>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/default_plugin/tools/move_tool.h>

#include "place_waypoint_tool.h"

namespace track_annotation_tool
{

// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
PlaceWaypointTool::PlaceWaypointTool() : moving_waypoint_node_( NULL ), current_waypoint_property_( NULL ), move_tool_( new rviz::MoveTool() )
{
  shortcut_key_ = 'w';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
PlaceWaypointTool::~PlaceWaypointTool()
{
  delete move_tool_;
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void PlaceWaypointTool::onInitialize()
{
  setName("Place waypoint (W)");
  move_tool_->initialize( context_ );

  first_activation_ = true;
  flag_resource_ = "package://track_annotation_tool/media/flag.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "PlaceWaypointTool: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

  moving_waypoint_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  moving_waypoint_node_->attachObject( entity );
  moving_waypoint_node_->setVisible( false );

  waypoint_publisher_ = nh_.advertise<geometry_msgs::PointStamped>( "track_annotation_tool/new_waypoints", 1 );
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_waypoint_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void PlaceWaypointTool::activate()
{
  if(first_activation_)
  {
    // Only show this message once at first activation, otherwise we always overwrite the messages by the DeleteWaypoint tool
    last_status_ = "Click to place waypoint at current position. Hold ALT to move camera (behave like \"Move\" tool).";
    setStatus(last_status_);
    first_activation_ = false;
  }

  if( moving_waypoint_node_ )
  {
    moving_waypoint_node_->setVisible( true );

    current_waypoint_property_ = new rviz::VectorProperty( "Current waypoint" );
    current_waypoint_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_waypoint_property_ );
  }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current waypoint
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of waypoints when
// we switch to another tool.
void PlaceWaypointTool::deactivate()
{
  if( moving_waypoint_node_ )
  {
    moving_waypoint_node_->setVisible( false );
    delete current_waypoint_property_;
    current_waypoint_property_ = NULL;
  }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int PlaceWaypointTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  // To allow zoom and pan
  if( event.alt() || event.middle() || event.right() || event.wheel_delta ) {
    move_tool_->processMouseEvent( event );

    // Reset status because move tool overwrites it
    setStatus(last_status_);
  }

  // If ALT key is pressed, we behave like a normal "move" tool
  if( event.alt() ) {
    moving_waypoint_node_->setVisible( false );
  }
  else {
    if( !moving_waypoint_node_ )
    {
      return Render;
    }

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection ))
    {
      moving_waypoint_node_->setVisible( true );
      moving_waypoint_node_->setPosition( intersection );
      current_waypoint_property_->setVector( intersection );

      if( event.leftDown() )
      {
        addWaypoint( intersection );
        return Render; // | Finished;
      }
    }
    else
    {
      moving_waypoint_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
    }
  }
  return Render;
}

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void PlaceWaypointTool::addWaypoint( const Ogre::Vector3& position )
{
  geometry_msgs::PointStamped ps;
  ps.point.x = position.x;
  ps.point.y = position.y;
  ps.point.z = position.z;
  ps.header.frame_id = context_->getFixedFrame().toStdString();
  ps.header.stamp = ros::Time::now();
  waypoint_publisher_.publish( ps );

  // Show status message
  std::stringstream ss;
  ss << "New waypoint has been added at coordinates " << std::fixed << std::setprecision(2) << "[" << position.x << ", " << position.y << "]. Hold ALT to move camera (behave like \"Move\" tool).";
  last_status_ = QString::fromStdString(ss.str());
  setStatus(last_status_);
}

} // end namespace track_annotation_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(track_annotation_tool::PlaceWaypointTool, rviz::Tool)
// END_TUTORIAL

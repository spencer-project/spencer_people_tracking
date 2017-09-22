^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spencer_people_tracking_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2017-09-22)
------------------
* Merge pull request `#2 <https://github.com/LCAS/spencer_people_tracking/issues/2>`_ from spencer-project/master
  Integrate multiple fixes from upstream
* Add missing dependencies for tracking_on_bagfile.launch
  map_server and openni2_launch were missing
* Merge pull request `#41 <https://github.com/LCAS/spencer_people_tracking/issues/41>`_ from LCAS/master
  1.0.7
* Contributors: Marc Hanheide, Timm Linder

1.0.7 (2017-06-09)
------------------
* Fix tracking_on_bagfile.launch to work in Docker container
* Contributors: Timm Linder

1.0.6 (2017-06-09)
------------------

1.0.5 (2017-05-12)
------------------

1.0.4 (2017-05-12)
------------------

1.0.3 (2017-05-11)
------------------

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* made it catkin package
* various install targets added that were missing
* Enable selection of PCL people detector in example launch files.
  Usage example added to the README file.
* Adding exemplary bagfile (downloaded via script) + launch file + Rviz config
* Remap upper-body detector topics correctly for tracking_single_rgbd_sensor.launch to work with OpenNi1, `#20 <https://github.com/LCAS/spencer_people_tracking/issues/20>`_
* Add option in tracking_single_rgbd_sensor.launch to start groundHOG detector if desired.
  Minor reordering of args and fix comment.
* Revert laser_detectors.launch to previous version, use original topic names
* Updated Rviz config for tracking_single_rgbd_sensor.launch
* Trying to fix tracking_single_rgbd_sensor.launch with Kinect v1
* Compiling on kinetic. Rviz crashes with views (QT bug?)
* Fix path to launch file, thanks for reporting @pirobot
* Minor comments added to launch files
* Updating launch files from SPENCER repo, now split up into Freiburg and RWTH tracking
* Updated licenses
* Rename ground_plane_really_fixed to ground_plane_fixed
  Old ground_plane_fixed.launch deleted (was there for historic reasons)
* Fix typo
* Adding launch script for dual leg_detectors
* Prerequisites for IMM version
  Includes new BasicOcclusionManager, IMMFilter, different motion models, restructured tracker launch file.
  IMM parameters still missing, therefore disabled for the moment.
  Author names added at the beginning of each file.
  Some cleanup.
* Updated RViz config, fix wrong Z position of bounding boxes
* Increasing process noise level until IMM version is pushed
* Synching topic names with internal repo, changes from integration week III
* Renaming RViz config file
* Adding package spencer_people_tracking_launch with exemplary launch files
* Contributors: Joao Avelino, Marc Hanheide, Timm Linder

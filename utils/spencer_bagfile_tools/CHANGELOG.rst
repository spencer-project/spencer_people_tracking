^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spencer_bagfile_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2020-08-28)
------------------

1.3.0 (2020-08-26)
------------------
* Merge branch 'melodic' into noetic
* Changes required for compatibility with ROS Noetic
  - Migrate scripts from Python 2.7 to Python 3 via 2to3 converter + manual editing
  - Update package dependencies from Python 2 to Python 3
* Contributors: Timm Linder

1.2.0 (2020-08-26)
------------------
* Merge branch 'master' into melodic
* Fixes required for ROS Melodic support
  - OpenCV2 to 3 migration
  - Add boost namespace for shared_ptr
* Contributors: Timm Linder

1.0.11 (2020-08-26)
-------------------
* Contributors: Timm Linder

1.0.10 (2018-09-22)
-------------------
* Merge pull request `#47 <https://github.com/LCAS/spencer_people_tracking/issues/47>`_ from LCAS/master
  1.0.8
* Contributors: Timm Linder

1.0.9 (2018-01-17)
------------------

1.0.8 (2017-09-22)
------------------
* Merge pull request `#41 <https://github.com/LCAS/spencer_people_tracking/issues/41>`_ from LCAS/master
  1.0.7
* Contributors: Timm Linder

1.0.7 (2017-06-09)
------------------

1.0.6 (2017-06-09)
------------------

1.0.5 (2017-05-12)
------------------

1.0.4 (2017-05-12)
------------------

1.0.3 (2017-05-11)
------------------
* moved tf_filter
* added nav_msgs and install targets
* Contributors: Marc Hanheide

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* homogenised all version strings to 1.0.0
* various install targets added that were missing
* Update to spencer_bagfile_tools, track annotation tool added
* Add some missing dependencies for rosdep, add missing srl_laser_segmentation dependency in srl_nearest_neighbor_tracker
* Make playback scripts and launch file more configurable
* In playback_from_robot.launch, optionally do not publish recorded tracked persons
* Compiling on kinetic. Rviz crashes with views (QT bug?)
* Fixes to spencer_bagfile_tools
* Adding package spencer_bagfile_tools
* Contributors: Joao Avelino, Marc Hanheide, Timm Linder

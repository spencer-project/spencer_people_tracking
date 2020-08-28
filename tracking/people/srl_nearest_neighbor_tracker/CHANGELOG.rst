^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srl_nearest_neighbor_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Timm Linder

1.0.11 (2020-08-26)
-------------------
* Various smaller fixes for ROS Kinetic
  - Use find_package(Eigen3) instead of find_package(Eigen)
  - Configurable odom and base footprint frame IDs (fixes `#53 <https://github.com/spencer-project/spencer_people_tracking/issues/53>`_)
* Revert added params for topic remapping introduced in pull request `#59 <https://github.com/spencer-project/spencer_people_tracking/issues/59>`_ (commits e5ca86b28c and 6255cfe76).
  The naming of these parameters is a bit inconsistent and support was added to only few selected ROS nodes. Therefore, it is better to rely on traditional topic remappings for now.
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
* Add missing install targets
* Contributors: Timm Linder

1.0.6 (2017-06-09)
------------------

1.0.5 (2017-05-12)
------------------

1.0.4 (2017-05-12)
------------------
* added angles dep
* Contributors: Marc Hanheide

1.0.3 (2017-05-11)
------------------

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* Add some missing dependencies for rosdep, add missing srl_laser_segmentation dependency in srl_nearest_neighbor_tracker
* Check for shared_ptr being defined in nnt
  Fixes `#19 <https://github.com/LCAS/spencer_people_tracking/issues/19>`_
* Adding tuning scripts
* Adding some test launch files
* Major code update of NNT to latest version
  * Multiple data association methods implemented
  * IMM updated
  * Occlusion handling methods added
  * Missed observation recovery (low confidence detections) now in separate class
  * Learned parameters added
  * Licenses updated and added to each source code file
* Prerequisites for IMM version
  Missing changes from commit c9d92b673e0ebf
* Prerequisites for IMM version
  Includes new BasicOcclusionManager, IMMFilter, different motion models, restructured tracker launch file.
  IMM parameters still missing, therefore disabled for the moment.
  Author names added at the beginning of each file.
  Some cleanup.
* Set trackedPerson.is_matched correctly in srl_nearest_neighbor_tracker
* Improved directory structure for clarity
* Contributors: Kota Weaver, Timm Linder

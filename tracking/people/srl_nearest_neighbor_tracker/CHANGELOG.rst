^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srl_nearest_neighbor_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

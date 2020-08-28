^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srl_laser_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2020-08-28)
------------------

1.3.0 (2020-08-26)
------------------
* Merge branch 'melodic' into noetic
* Changes required for compatibility with ROS Noetic
  - Migrate scripts from Python 2.7 to Python 3 via 2to3 converter + manual editing
  - Update package dependencies from Python 2 to Python 3
  - Retrain laser detector due to incompatibility of existing models with new OpenCV version
* Contributors: Timm Linder

1.2.0 (2020-08-26)
------------------
* Merge branch 'master' into melodic
* Contributors: Timm Linder

1.0.11 (2020-08-26)
-------------------
* Restore functionality of srl_laser_detectors by replacing old trained models (broken due to changes in Eigen and OpenCV) with new ones.
  Training functionality restored by slight revision of training ROS node according to new protocol.
  Improved error handling in laser detectors.
  Include boundary dist feature to ensure compatibility of trained models.
  Fix compiler warnings in getDescription() method of laser features.
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
* Merge pull request `#2 <https://github.com/LCAS/spencer_people_tracking/issues/2>`_ from spencer-project/master
  Integrate multiple fixes from upstream
* Allow remapping of laser topic in 2D laser detector and segmentation
* Merge pull request `#41 <https://github.com/LCAS/spencer_people_tracking/issues/41>`_ from LCAS/master
  1.0.7
* Contributors: Marc Hanheide, Timm Linder

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

1.0.3 (2017-05-11)
------------------
* added missing dep
* Contributors: Marc Hanheide

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* Updating licenses for laser detector, features and segmentation
* Updating laser detectors, segmentation and features from SRL SVN.
  New random forest detector implemented, extended jump distance segmentation added, bugfix for feature 08, other various fixes and improvements.
* Adding SRL laser detectors and segmentation
* Contributors: Timm Linder

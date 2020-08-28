^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcl_people_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2020-08-28)
------------------

1.3.0 (2020-08-26)
------------------
* Merge branch 'melodic' into noetic
* Contributors: Timm Linder

1.2.0 (2020-08-26)
------------------
* Merge branch 'master' into melodic
* Fixes required for ROS Melodic support
  - OpenCV2 to 3 migration
* Contributors: Timm Linder

1.0.11 (2020-08-26)
-------------------
* Use find_package(Eigen3) instead of find_package(Eigen)
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
* added visualisation_msgs dep
* Contributors: Marc Hanheide

1.0.2 (2017-05-09)
------------------

1.0.1 (2017-05-09)
------------------
* homogenised all version strings to 1.0.0
* various install targets added that were missing
* Enable selection of PCL people detector in example launch files.
  Usage example added to the README file.
* Fix vertical flip of RGB images in PCL people detector (hope this works in all cases/sensor setups)
* Add missing OpenCV 2 dependencies
* Fix rosdep issue due to typo
* Compiling on kinetic. Rviz crashes with views (QT bug?)
* Adding launch file for PCL people detector
* Update README.md
* Adding ground-based RGB-D people detector from PCL library
* Contributors: Joao Avelino, Marc Hanheide, Timm Linder

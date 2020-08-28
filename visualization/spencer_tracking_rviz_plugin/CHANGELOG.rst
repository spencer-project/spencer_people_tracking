^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spencer_tracking_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2020-08-28)
------------------
* Fix Qt5 cmake module not being found
* Contributors: Timm Linder

1.3.0 (2020-08-26)
------------------
* Merge branch 'melodic' into noetic
* Changes required for compatibility with ROS Noetic
  - Migrate scripts from Python 2.7 to Python 3 via 2to3 converter + manual editing
  - Update package dependencies from Python 2 to Python 3
  - Update Dockerfiles for Noetic, add extra required dependencies
* Contributors: Timm Linder

1.2.0 (2020-08-26)
------------------
* Merge branch 'master' into melodic
* Fixes required for ROS Melodic support
  - Replace deprecated pluginlib macro
  - Qt5 support for Rviz plugins
  - Replace tf by tf2 message_filter in rviz plugin (contributed by user @f-fl0 in srl-freiburg/spencer_tracking_rviz_plugin)
  - Add boost namespace for shared_ptr (for rviz plugin, this was contributed by @makokal in srl-freiburg/spencer_tracking_rviz_plugin)
* Contributors: Timm Linder, Billy Okal, f-fl0

1.0.11 (2020-08-26)
-------------------
* Various smaller fixes for ROS Kinetic
  - Use find_package(Eigen3) instead of find_package(Eigen)
  - Configurable odom and base footprint frame IDs (fixes `#53 <https://github.com/spencer-project/spencer_people_tracking/issues/53>`_)
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
* Specify correct license in package.xmls
* Merge pull request `#41 <https://github.com/LCAS/spencer_people_tracking/issues/41>`_ from LCAS/master
  1.0.7
* Contributors: Marc Hanheide, Timm Linder

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

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* homogenised all version strings to 1.0.0
* Fixed segfault. You shouldnt mix QT4 and QT5
* Compiling on kinetic. Rviz crashes with views (QT bug?)
* Updating lots of utility packages to latest version from SPENCER repo. Licenses updated.
* Ignore Z position in visualization by default
* Update README.md
* Update README.md
* Disable group history visualization per default since it is very CPU-intensive, reduce default history size
* Various fixes to TrackedPersons and DetectedPersons display in RViz
  - Fix popping up of track / detection IDs when persons first appear
  - Add option to ignore Z position from ROS message
  - Prevent crash when covariance matrix is not positive (semi-)definite
  - Add option to hide IDs of single-person groups
  - Display MISSED (new) / MATCHED / OCCLUDED track status to distinguish between misdetections and physical occlusions
* Update README.md
* Adding message definitions, RViz plugins and mock scripts
* Contributors: Joao Avelino, Marc Hanheide, Timm Linder

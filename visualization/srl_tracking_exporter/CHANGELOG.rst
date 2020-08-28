^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package srl_tracking_exporter
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
* Contributors: Timm Linder

1.0.11 (2020-08-26)
-------------------
* Merge pull request `#65 <https://github.com/spencer-project/spencer_people_tracking/issues/65>`_ from dandedrick/fix-logerror
  job_monitor: replace logerror with logerr
* job_monitor: replace logerror with logerr
  rospy doesn't export a logerror symbol only logerr. If this was ever hit it
  would have caused an exception instead of logging.
* Contributors: Dan Dedrick, Timm Linder

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

1.0.2 (2017-05-09)
------------------
* added missing roslib deps
* Contributors: Marc Hanheide

1.0.1 (2017-05-09)
------------------
* homogenised all version strings to 1.0.0
* Updating lots of utility packages to latest version from SPENCER repo. Licenses updated.
* Adding SVG exporter
* Contributors: Marc Hanheide, Timm Linder

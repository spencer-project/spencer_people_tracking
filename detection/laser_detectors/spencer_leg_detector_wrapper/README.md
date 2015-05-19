#### SPENCER wrapper for the laser-based leg detector from wg-perception

This package provides a launch file for integrating the laser-based `leg_detector` package from [`wg_perception/people`](https://github.com/wg-perception/people)
into the SPENCER people tracking framework. The `leg_detector` must be provided separately; it is a third-party component not part of the SPENCER repository.

##### Usage

Run

    roslaunch spencer_leg_detector_wrapper leg_detector.launch laser:=/spencer/sensors/laser_front/echo0 name:=legs_laser_front 

##### Credits

The wrapper is ©2015 Timm Linder, Social Robotics Laboratory, University of Freiburg

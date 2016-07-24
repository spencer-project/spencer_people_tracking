spencer_bagfile_tools package
=============================

Usage
-----
For playing back bagfiles, use

    roslaunch spencer_bagfile_tools playback.launch folder:="/media/spencer/june2014/06-gate-f6-san-fran"

For supported arguments, see the beginning of playback.launch. Not all sensor modalities are enabled by default. 
To use rosbag play instead of rqt_bag for playback, specify rqt:=false in the roslaunch command line.
Set visualization:=false to disable automatic start of rviz.


More example command-lines
--------------------------

    roslaunch spencer_bagfile_tools playback.launch folder:="/media/spencer/june2014/09-terminal-d-pedestrian-flow-and-shops" rqt:=false depth:=false kinect2:=false front_rgbd:=false rear_rgbd:=false dslr:=true start:=650 duration:=50 rate:=0.1


Note about rqt_bag
------------------
When using rqt_bag, don't forget to right-click and select the topics to publish, otherwise nothing will happen!
rqt_bag, as opposed to rosbag play, can give a preview of camera image thumbnails and allows to move back and forth
in the bagfile.


(C) 2013-2015 Timm Linder, Social Robotics Lab, Uni Freiburg
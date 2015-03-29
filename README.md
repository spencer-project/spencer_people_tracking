### SPENCER Multi-Modal People Detection & Tracking Framework
###### Multi-modal ROS-based people and group detection & tracking framework for mobile robots developed within the context of the EU FP7 project [SPENCER](http://www.spencer.eu).

![Tracked persons projected into the front RGB-D camera](/../screenshots/screenshots/full-system-3-front-cam-edited.png?raw=true "Tracked persons projected into the front RGB-D camera")


*More code, documentation and videos will be added shortly.*

#### Motivation

The aim of the EU FP7 research project SPENCER is to develop algorithms for service robots that can guide groups of people through highly dynamic and crowded pedestrian environments, such as airports or shopping malls, while behaving in a socially compliant manner by e.g. not crossing in between families or couples. Exemplary situations that such a robot could encounter are visualized in below image on the right. To this end, robust and computationally efficient components for the perception of humans in the robot's surroundings need to be developed.

![SPENCER Use-Case](/../screenshots/screenshots/spencer-use-case-2.jpg?raw=true "SPENCER Use-Case") &nbsp;&nbsp; ![SPENCER Use-Case](/../screenshots/screenshots/use-case-1-bw.png?raw=true "SPENCER Use-Case")


#### Architecture

The following figure shows the real-time people and group detection and tracking pipeline developed in the context of the SPENCER project:

![SPENCER Tracking Pipeline](/../screenshots/screenshots/data-flow-with-layers-bw.png?raw=true "SPENCER Tracking Pipeline")

The entire communication between different stages of our pipeline occurs via ROS messages which encourage reuse of our components in custom setups. The modular architecture allows for easy interchangeability of individual components in all stages of the pipeline.

#### Overview of available components

##### Message definitions

We provide a set of reusable ROS message type definitions, which we have successfully applied across various people detection and tracking scenarios, over different sensor modalities and tracking approaches. Most relevant messages can be found inside the [spencer_tracking_msgs](/messages/spencer_tracking_msgs) package. 

We highly encourage reuse of these messages to benefit from our rich infrastructure of detection, tracking, filtering and visualization components! Existing detection and tracking algorithms can often easily be integrated by publishing additional messages in our format, or by writing a simple C++ or Python node that converts between message formats.


##### People detection

We have integrated the following person detection modules:
- A reimplementation of a boosted [2D laser segment classifier](/detection/laser_detectors/srl_laser_detectors), based upon the method by Arras et al. [3]
- An [RGB-D upper-body detector](/detection/rgbd_detectors/rwth_upper_body_detector) described more closely in [2], which slides a normalized depth template over ROIs in the depth image
- A monocular-vision full-body HOG detector ([groundHOG](/detection/monocular_detectors/rwth_ground_hog)) [2], which based upon a given ground plane estimate determines the image corridor in which pedestrians can be expected. This detector is GPU-accelerated using CUDA. The contained [cudaHOG](/detection/monocular_detectors/3rd_party) library requires manual compilation and a recent CUDA SDK as well as an nVidia graphics card.
- An RGB-D detector from the [PCL library](http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php), which extracts candidate ROIs on a groundplane and then applies a linear HOG classifier [4] *(code will be added shortly)*

##### Multi-modal detection and fusion

For detection-to-detection fusion, we have implemented a series of nodelets which can be used to flexibly compose a fusion pipeline by means of roslaunch XML files. Details can be found in the [spencer_detected_person_association](/detection/spencer_detected_person_association) package. The following figure shows an example configuration which was used during experiments in SPENCER:

![Example multi-modal people tracking architecture](/../screenshots/screenshots/multimodal-architecture.png?raw=true "Example multi-modal people tracking architecture")

In case of detection-to-track fusion (currently not implemented), it is still advisable to publish a [CompositeDetectedPerson](/messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg) message (via [CompositeDetectedPersons](/messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg)) for each set of detections associated with a track, such that later on it is possible to go back to the original detections from a track, and lookup associated image bounding boxes etc. via the associated detection_id.

##### Person and group tracking
For person and group tracking, we currently provide exemplary code based upon a nearest-neighbor standard filter data association, which is robust enough in most use cases (especially if multi-modal detectors are being used). The [person tracker](/tracking/people/srl_nearest_neighbor_tracker) has been enhanced with a track initiation logic and 4 different IMM-based motion models (constant velocity with low process noise, high process noise, coordinated turn and Brownian motion) to make tracking more robust. The [group tracker](/tracking/groups/spencer_group_tracking) relies on [social/spatial relations](/tracking/groups/spencer_social_relations) determined via the same coherent motion indicator features as described in [1].

Internally, we have already integrated more advanced methods, including a track-oriented multi-hypothesis person tracker [2], and a hypothesis-oriented multi-model multi-hypothesis person and group tracker [1]. These components use exactly the same ROS message definitions, however, they are not yet publicly available. The components available here were originally implemented as baseline methods for comparison.

##### Filtering of tracked persons & tracking metrics

The [spencer_tracking_utils](/tracking/people/spencer_tracking_utils) package contains a number of standalone ROS nodes that can filter an incoming set of [TrackedPerson](/messages/spencer_tracking_msgs/msg/TrackedPerson.msg) messages based upon different criteria, e.g. distance to the sensor/robot, visually confirmed tracks only, etc.

In [spencer_tracking_metrics](/tracking/people/spencer_tracking_metrics), we have wrapped publicly available implementations of different tracking metrics, such as CLEAR-MOT and OSPA, such that they are compatible with our message definitions. These are useful for evaluating tracking performance for a given groundtruth.

##### Import of old annotated logfiles

The [srl_tracking_logfile_import](/tracking/people/srl_tracking_logfile_import) package provides a Python script for importing old 2D laserscan logfiles in CARMEN format that have been annotated with groundtruth person tracks, such as [these datasets](http://www2.informatik.uni-freiburg.de/~luber/people_tracker/logfiles/logfiles.html).

##### Visualization

The [srl_tracking_exporter](/visualization/srl_tracking_exporter) package contains a useful Python script for rendering track trajectories, detections and robot odometry from a 2D top-down perspective as scalable vector graphics (SVGs). These can optionally be animated to visualize the evolution of one or multiple tracks over time.

One major highlight of our framework is a reusable and highly configurable set of [custom RViz plugins](/visualization/spencer_tracking_rviz_plugin) for the visualization of:
- Detected persons
- Tracked persons (including occlusion state, associated detection ID, and covariance ellipses)
- Social relations
- Tracked groups

As an example, some features of the tracked persons display are:
- Different visual styles: 3D bounding box, cylinder, animated human mesh
- Coloring: 6 different color palettes
- Display of velocity arrows
- Visualization of the 99% covariance ellipse
- Display of track IDs, status (matched, occluded), associated detection IDs
- Configurable reduction of opacity when a track is occluded
- Track history (trajectory) display as dots or lines
- Configurable font sizes and line widths

All of the following screenshots have been generated using these plugins.

#### Example screenshots of our system in action

The following screenshots show our system in action, while playing back recorded data from a crowded airport environment:

###### Multi-modal people detection. In orange: 2D laser [3], cyan: upper-body RGB-D [2], yellow: monocular vision HOG [2], grey: fused detections (when using detection-to-detection fusion).
![Example detection results](/../screenshots/screenshots/full-system-3-detections-crop2.png?raw=true "Example detection results")

###### People tracking. In red: tracks visually confirmed by image-based detector
![Example people tracking results](/../screenshots/screenshots/full-system-3-tracks-crop2.png?raw=true "Example people tracking results")

###### Group tracking via coherent motion indicator features, as described in [1]
![Example group tracking results](/../screenshots/screenshots/full-system-3-groups-crop2.png?raw=true "Example group tracking results")



#### Demo videos

Videos of the people detection and tracking system in action can be found on the [SPENCER YouTube Channel](https://www.youtube.com/user/spencereuproject):

- [Single Person Guidance Scenario Prototype](https://www.youtube.com/watch?v=DQm55LLmvgg) (2D laser only)
- [Group Guidance Scenario Prototype](https://www.youtube.com/watch?v=V5PYFf9A-PU) (2D laser only)
- ... (more to come soon!)

#### Runtime performance

On the SPENCER robot platform, which is equipped with front and rear RGB-D sensors (Asus Xtion Pro Live) and two SICK LMS500 laser scanners, we distributed the people and group detection & tracking system over two high-end gaming laptops (Intel Core i7-4700MQ, nVidia GeForce 765M). The detectors for the frontal sensors were executed on one laptop along with the detection-fusion pipeline. The detectors for the rear-facing sensors and the people and group tracking modules were executed on the second laptop. Both laptops were connected with each other and the rest of the platform via gigabit ethernet.

With this configuration, the components run in real-time at 20-25 Hz (with visualization off-loaded to a separate computer), even in crowded environments where more than 30 persons are concurrently visible.

#### Installation

The people and group detection and tracking framework has been tested on Ubuntu 12.04 using ROS Hydro, as well as Ubuntu 14.04 using ROS Indigo. We recommend installation of ROS and the required depencencies of our components via:

##### Required dependencies

###### Using ROS Hydro on Ubuntu 12.04 (Precise)

    sudo apt-get install ros-hydro-desktop-full
    sudo apt-get install libeigen3-dev libsvm-dev python-numpy python-scipy ros-hydro-openni-launch ros-hydro-openni2-launch ros-hydro-cmake-modules ros-hydro-eigen-conversions

###### Using ROS Indigo on Ubuntu 14.04 (Trusty)

    sudo apt-get install ros-indigo-desktop-full
    sudo apt-get install libeigen3-dev libsvm-dev python-numpy python-scipy ros-indigo-openni-launch ros-indigo-openni2-launch ros-indigo-cmake-modules ros-indigo-eigen-conversions
##### Building our ROS packages

As we currently do not yet provide any pre-built Debian packages, we suggest to [create a new catkin workspace](http://wiki.ros.org/catkin/workspaces) for our framework, and then clone the content of this repository into the `src` folder of this new workspace. Then, build the workspace using the normal methods (catkin_make / catkin build).

##### Note on CUDA SDK for the groundHOG detector

The cudaHOG library used by the groundHOG detector requires an nVidia graphics card and an installed CUDA SDK (recommended version: 6.5). As installing CUDA (especially on laptops with Optimus/Bumblebee) and compiling the library is not straightforward, detailled installation instructions are provided [here](/detection/monocular_detectors/3rd_party). Once these instructions have been followed, the `rwth_ground_hog` package needs to be rebuilt using catkin. If no CUDA SDK is installed, the ROS package will still compile, but it will not provide any functionality.


#### Credits, license & how to cite

The software in this repository is maintained by:

- [Timm Linder](http://www.timmlinder.com), Social Robotics Lab, Albert-Ludwigs-Universität Freiburg
- [Stefan Breuers](http://www.vision.rwth-aachen.de/people/stefan-breuers), Computer Vision Group, RWTH Aachen University

This work has been supported by the EC under contract number FP7-ICT-600877 (SPENCER).
If you use the software contained in this repository for your research, please cite the following publication:

> TBD

Most of the software in this repository is released under a BSD (2-clause) license. For details, however, please check the individual ROS packages.


#### References

[1] Linder T. and Arras K.O. *Multi-Model Hypothesis Tracking of Groups of People in RGB-D Data.* IEEE Int. Conference on Information Fusion (FUSION'14), Salamanca, Spain, 2014.

[2] Jafari O. Hosseini and Mitzel D. and Leibe B.. *Real-Time RGB-D based People Detection and Tracking for Mobile Robots and Head-Worn Cameras*. IEEE International Conference on Robotics and Automation (ICRA'14), 2014.

[3] Arras K.O. and Martinez Mozos O. and Burgard W.. *Using Boosted Features for the Detection of People in 2D Range Data*. IEEE International Conference on Robotics and Automation (ICRA'07), Rome, Italy, 2007.

[4] Munaro M. and Menegatti E. *Fast RGB-D people tracking for service robots*. In Autonomous Robots, Volume 37 Issue 3, pp. 227-242, Springer, 2014.

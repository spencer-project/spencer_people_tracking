### SPENCER Multi-Modal People Detection & Tracking Framework &nbsp;&nbsp;&nbsp;[![Build Status](https://lcas.lincoln.ac.uk/buildfarm/buildStatus/icon?job=Kbin_uX64__spencer_people_tracking_full__ubuntu_xenial_amd64__binary)](https://lcas.lincoln.ac.uk/buildfarm/job/Kbin_uX64__spencer_people_tracking_full__ubuntu_xenial_amd64__binary)

###### Multi-modal ROS-based people and group detection & tracking framework for mobile robots developed within the context of the EU FP7 project [SPENCER](http://www.spencer.eu).

![Tracked persons projected into the front RGB-D camera](/../screenshots/screenshots/full-system-3-front-cam-edited.png?raw=true "Tracked persons projected into the front RGB-D camera")

#### Features at a glance

- **Multi-modal detection:** Multiple RGB-D & 2D laser detectors in one common framework.
- **People tracking:** Efficient tracker based upon nearest-neighbor data association.
- **Social relations:** Estimate spatial relations between people via coherent motion indicators.
- **Group tracking:** Detection and tracking of groups of people based upon their social relations.
- **Robustness:** Various extensions such as IMM, track initiation logic and high-recall detector input make the people tracker work relatively robustly even in very dynamic environments.
- **Real-time:** Runs at 20-30 Hz on gaming laptops, tracker itself requires only ~10% of 1 CPU core.
- **Extensible and reusable:** Well-structured ROS message types and clearly defined interfaces make it easy to integrate custom detection and tracking components.
- **Powerful visualization:** A series of reusable RViz plugins that can be configured via mouse click, plus scripts for generating animated (2D) SVG files.
- **Evaluation tools:** Metrics (CLEAR-MOT, OSPA) for evaluation of tracking performance.
- **ROS integration:** All components are fully integrated with ROS and written in C++ or Python. No Matlab required.

#### Motivation

The aim of the EU FP7 research project SPENCER was to develop algorithms for service robots that can guide groups of people through highly dynamic and crowded pedestrian environments, such as airports or shopping malls, while behaving in a socially compliant manner by e.g. not crossing in between families or couples. Exemplary situations that such a robot could encounter are visualized in below image on the right. To this end, robust and computationally efficient components for the perception of humans in the robot's surroundings needed to be developed.

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
- A reimplementation of a **[boosted 2D laser segment classifier](/detection/laser_detectors/srl_laser_detectors)**, based upon the method by Arras et al. [3]
- An **[RGB-D upper-body detector](/detection/rgbd_detectors/rwth_upper_body_detector)** described more closely in [2], which slides a normalized depth template over ROIs in the depth image
- A monocular-vision **[full-body HOG detector (groundHOG)](/detection/monocular_detectors/rwth_ground_hog)** [2], which based upon a given ground plane estimate determines the image corridor in which pedestrians can be expected. This detector is GPU-accelerated using CUDA. The contained [cudaHOG](/detection/monocular_detectors/3rd_party) library requires manual compilation and a recent CUDA SDK as well as an nVidia graphics card.
- An **[RGB-D detector from PCL](/detection/rgbd_detectors/pcl_people_detector)**, which extracts candidate ROIs on a groundplane and then applies a linear HOG classifier [4]

Further external detectors which output `geometry_msgs/PoseArray` or `people_msgs/PositionMeasurementArray` messages can easily be integrated into our framework using the scripts from [this package](/detection/spencer_detected_person_conversion). Examples of such detectors include:
- The **[laser-based leg detector](https://github.com/wg-perception/people/tree/indigo-devel/leg_detector)** from wg-perception, which might work better than our own laser detector if the sensor is located very close to the ground. See our [wrapper package](/detection/laser_detectors/spencer_leg_detector_wrapper/) and [`leg_detectors.launch`](/launch/spencer_people_tracking_launch/launch/detectors/leg_detectors.launch) (replaces `laser_detectors.launch`).

##### Multi-modal detection and fusion

For detection-to-detection fusion, we have implemented a series of nodelets which can be used to flexibly compose a fusion pipeline by means of roslaunch XML files. Details can be found in the [spencer_detected_person_association](/detection/spencer_detected_person_association) package. The following figure shows an example configuration which was used during experiments in SPENCER:

![Example multi-modal people tracking architecture](/../screenshots/screenshots/multimodal-architecture.png?raw=true "Example multi-modal people tracking architecture")

In case of detection-to-track fusion (currently not implemented), it is still advisable to publish a [CompositeDetectedPerson](/messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg) message (via [CompositeDetectedPersons](/messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg)) for each set of detections associated with a track, such that later on it is possible to go back to the original detections from a track, and lookup associated image bounding boxes etc. via the associated detection_id.

##### Person and group tracking
For person and group tracking, we currently provide exemplary code based upon a nearest-neighbor standard filter data association, which is robust enough in most use cases (especially if multi-modal detectors are being used). The **[people tracker](/tracking/people/srl_nearest_neighbor_tracker)** has been enhanced with a track initiation logic and 4 different IMM-based motion models (constant velocity with low process noise, high process noise, coordinated turn and Brownian motion) to make tracking more robust.

The [group tracker](/tracking/groups/spencer_group_tracking) relies on [social/spatial relations](/tracking/groups/spencer_social_relations) determined via the same coherent motion indicator features as described in [1].

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

- [Real-Time Multi-Modal People Tracking in a Crowded Airport Environment](https://www.youtube.com/watch?v=r2fGCrekGj8) (RGB-D and 2D laser)
- [Single Person Guidance Scenario Prototype](https://www.youtube.com/watch?v=DQm55LLmvgg) (2D laser only)
- [Group Guidance Scenario Prototype](https://www.youtube.com/watch?v=V5PYFf9A-PU) (2D laser only)


#### Runtime performance

On the SPENCER robot platform, which is equipped with front and rear RGB-D sensors (Asus Xtion Pro Live) and two SICK LMS500 laser scanners, we distributed the people and group detection & tracking system over two high-end gaming laptops (Intel Core i7-4700MQ, nVidia GeForce 765M). The detectors for the frontal sensors were executed on one laptop along with the detection-fusion pipeline. The detectors for the rear-facing sensors and the people and group tracking modules were executed on the second laptop. Both laptops were connected with each other and the rest of the platform via gigabit ethernet.

With this configuration, the components run in real-time at 20-25 Hz (with visualization off-loaded to a separate computer), even in crowded environments where more than 30 persons are concurrently visible.

#### Installation from L-CAS package repository

A packaged version of the entire framework for *ROS Kinetic* on *Ubuntu 16.04 (Xenial)* is kindly provided by the Lincoln Research Centre for Autonomous Systems (L-CAS) and built by their continuous integration system. You must first add their package repository to your apt/sources.list [as described here](https://github.com/LCAS/rosdistro/wiki#using-the-l-cas-repository-if-you-just-want-to-use-our-software). Then, install the framework via

    sudo apt-get install ros-kinetic-spencer-people-tracking-full
    
Note that the `groundHOG` detector in the packaged version is non-functional.

#### Installation from source

The people and group detection and tracking framework has been tested on Ubuntu 14.04 / ROS Indigo and Ubuntu 16.04 / ROS Kinetic. For more information on the Robot Operating System (ROS), please refer to [ros.org](http://www.ros.org/).

*NOTE: The entire framework only works on 64-bit systems. On 32-bit systems, you will encounter Eigen-related alignment issues (failed assertions). See issue [#1](https://github.com/spencer-project/spencer_people_tracking/issues/1)*

##### 1. Cloning the source code repository

As we currently do not yet provide any pre-built Debian packages, you have to build our framework from source code. As a first step, create a folder for a new catkin workspace and clone the GitHub repository into the `src` subfolder:

    cd ~/Code
    mkdir -p spencer-people-tracking-ws/src
    cd spencer-people-tracking-ws/src
    git clone https://github.com/spencer-project/spencer_people_tracking.git
    
##### 2. Installing required dependencies

Assuming you have ROS Indigo or ROS Kinetic already installed, we recommend installing the required depencencies of our framework via:

    rosdep update
    rosdep install -r --from-paths . --ignore-src
    
##### 3. Initializing the catkin workspace
 
Next, we suggest to use `catkin` (available via `sudo apt-get install python-catkin-tools`) to setup the workspace:

    cd ..
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    
##### 4. Building the ROS packages

Finally, build all packages via:

    catkin build -c -s

##### 5. Sourcing the ROS workspace

After building the workspace, source it via:

    source devel/setup.bash


##### Special note on CUDA SDK for the groundHOG detector

The cudaHOG library used by `rwth_ground_hog` requires an nVidia graphics card and an installed CUDA SDK (recommended version: 6.5). As installing CUDA (especially on laptops with Optimus/Bumblebee) and compiling the library is not straightforward, detailled installation instructions are provided [here](/detection/monocular_detectors/3rd_party). Once these instructions have been followed, the `rwth_ground_hog` package needs to be rebuilt using catkin. If no CUDA SDK is installed, the ROS package will still compile, but it will not provide any functionality.

#### Quick start tutorial

The following demo and three tutorials help you to easily get started using our framework.

##### Demo: Multimodal tracking in RGB-D and 2D laser from a bagfile

A short exemplary bagfile with 2D laser and RGB-D sensor data to test our framework can be downloaded by running

    rosrun spencer_people_tracking_launch download_example_bagfiles.sh

Then, you can launch

    roslaunch spencer_people_tracking_launch tracking_on_bagfile.launch
    
which will start playing back a bagfile (as soon as you unpause by pressing `SPACE`) and run Rviz for visualization.

###### Using the PCL people detector instead of the upper-body detector ######

As an alternative to the depth template-based upper-body detector, you can choose to use our slightly modified version of the person detector from the Point Cloud Library. This detector first performs a Euclidean clustering and head subcluster extraction, before validating the candidate ROIs using a HOG SVM. To do so, pass `use_pcl_detector:=true` to the launch file.

###### Enabling the groundHOG detector ######

If you have compiled the cudaHOG library (see description further above), you can optionally enable the groundHOG detector by passing `use_hog_detector:=true` to the launch file. The detection-to-detection fusion pipeline will then automatically fuse the detections from both detectors.


##### Tutorial 1: People / group tracking and visualization with a single RGB-D sensor

This is the easiest way to get started using just a single RGB-D sensor connected locally to your computer. Place your Asus Xtion Pro Live sensor horizontally on a flat surface, and connect it to your computer (or play the example bagfile linked in a section further below). Then run the following launch file from your people tracking workspace (make sure that you have sourced it, e.g. `source devel/setup.bash`):

    roslaunch spencer_people_tracking_launch tracking_single_rgbd_sensor.launch height_above_ground:=1.6
    
This will do the following:
- Start the OpenNi2 drivers (for Asus Xtion Pro) and publish RGB-D point clouds in the `/spencer/sensors/rgbd_front_top/` camera namespace
- Run an upper-body RGB-D detector, assuming a horizontal ground plane at 1.6 meters below the sensor. Other heights may work as well, but the detector has been trained at approximately this height.
- Run a simple detection-to-detection fusion pipeline
- Run the `srl_nearest_neighbor_tracker`, which will subscribe to `/spencer/perception/detected_persons` and publish tracks at `/spencer/perception/tracked_persons`
- Run RViz with a predefined configuration, which shows the point cloud, the sensor's view frustum, and detected and tracked persons (using our custom RViz plugins).

###### Using MS Kinect v1 ######

The original MS Kinect v1 sensor does not support OpenNi2. In this case, append `use_openni1:=true` to the above command-line of the launch file to fall back to OpenNi1.


###### Troubleshooting ######

If you cannot see any detection bounding boxes, first check if the point cloud is displayed properly in RViz. If not, there is probably a problem with your RGB-D sensor (USB or OpenNi issues).


##### Tutorial 2: Tracking with front and rear laser + RGB-D sensors

To try out a sensor configuration similar to the SPENCER robot platform, run:

    roslaunch spencer_people_tracking_launch tracking_on_robot.launch
    
This assumes the RGB-D sensors mounted horizontally at about 1.6m above ground, and sensor data to be published on the following topics:

    /spencer/sensors/laser_front/echo0  [sensor_msgs/LaserScan]
    /spencer/sensors/laser_rear/echo0   [sensor_msgs/LaserScan]
    /spencer/sensors/rgbd_front_top/{rgb/image_raw,depth/image_rect}  [sensor_msgs/Image]
    /spencer/sensors/rgbd_front_top/{rgb/camera_info} [sensor_msgs/CameraInfo]
    /spencer/sensors/rgbd_rear_top/{rgb/image_raw,depth/image_rect}   [sensor_msgs/Image]
    /spencer/sensors/rgbd_rear_top/{rgb/camera_info}  [sensor_msgs/CameraInfo]

The launch file starts a pipeline similar to that from tutorial 1 (above), but includes a second set of RGB-D detectors for the rear sensor, as well as person detectors for the two 2D laser scanners. Sensor drivers which publish the RGB-D and laser data listed above are _not_ started automatically by this launch file. Also, you manually have to start Rviz.

###### Using a subset of these sensors

Note that the fusion pipeline reconfigures automatically if only a subset of the person detectors is running. If e.g. you don't have a rear RGB-D sensor, just comment out the line which includes `rear_rgbd_detectors.launch` in `tracking_on_robot.launch`.

##### Tutorial 3: Fully customized sensor configuration

1. Start your own launch files for starting person detectors, or use a combination of the launch files we provide in [`spencer_people_tracking_launch/launch/detectors`](/launch/spencer_people_tracking_launch/launch/detectors). You may have to remap input and output topics as needed.
2. Create a copy of [`detection_to_detection_fusion_pipeline.launch`](/detection/spencer_detected_person_association/launch/detection_to_detection_fusion_pipeline.launch) and its children, such as [`fuse_lasers_and_rgbd.launch`](detection/spencer_detected_person_association/launch/fuse_lasers_and_rgbd.launch), in [`spencer_detected_person_association`](/detection/spencer_detected_person_association). Based upon the provided example, create your own pipeline that step-by-step fuses detections from different detectors. For more information, see the corresponding package.
3. Create a copy of the `freiburg_people_tracking.launch` file in `spencer_people_tracking_launch`. Adjust it to refer to your own fusion launch file created in step 2.
3. Start your copy of `freiburg_people_tracking.launch`.
4. If needed, start group tracking via `roslaunch spencer_people_tracking_launch group_tracking.launch`.


#### Multi-modal datasets for evaluation

The multi-modal "Motion Capture" sequence from our ICRA 2016 paper is available upon request to let you evaluate your own detection / tracking algorithms on our dataset. For a fair comparison, please use the CLEAR-MOT metrics implementation contained in this repository, if possible. The raw data from the airport sequence cannot be shared for privacy reasons, though we might provide the extracted detections at a later point.


#### Credits, license & how to cite

The software in this repository is maintained by:

- [Timm Linder](http://www.timmlinder.com), Social Robotics Lab, Albert-Ludwigs-Universität Freiburg
- [Stefan Breuers](http://www.vision.rwth-aachen.de/people/stefan-breuers), Computer Vision Group, RWTH Aachen University

Credits of the different ROS packages go to the particular authors listed in the respective `README.md` and `package.xml` files. See also the `AUTHORS.md` file for a list of further contributors.

This work has been supported by the European Commission under contract number FP7-ICT-600877 (SPENCER), and has received additional funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 732737 (ILIAD).

If you use the software contained in this repository for your research, please cite the following publication:

> On Multi-Modal People Tracking from Mobile Platforms in Very Crowded and Dynamic Environments    
> Linder, T., Breuers, S., Leibe, B., Arras, K.O.    
> IEEE International Conference on Robotics and Automation (ICRA) 2016    

also optionally:

> People Detection, Tracking and Visualization using ROS on a Mobile Service Robot    
> Linder, T. and Arras, K.O.    
> Robot Operating System (ROS): The Complete Reference (Vol. 1).   
> Springer Studies in Systems, Decision and Control, 2016   

Most of the software in this repository is released under a BSD license. For details, however, please check the individual ROS packages.


#### References

[1] Linder T. and Arras K.O. *Multi-Model Hypothesis Tracking of Groups of People in RGB-D Data.* IEEE Int. Conference on Information Fusion (FUSION'14), Salamanca, Spain, 2014.

[2] Jafari O. Hosseini and Mitzel D. and Leibe B.. *Real-Time RGB-D based People Detection and Tracking for Mobile Robots and Head-Worn Cameras*. IEEE International Conference on Robotics and Automation (ICRA'14), 2014.

[3] Arras K.O. and Martinez Mozos O. and Burgard W.. *Using Boosted Features for the Detection of People in 2D Range Data*. IEEE International Conference on Robotics and Automation (ICRA'07), Rome, Italy, 2007.

[4] Munaro M. and Menegatti E. *Fast RGB-D people tracking for service robots*. In Autonomous Robots, Volume 37 Issue 3, pp. 227-242, Springer, 2014.

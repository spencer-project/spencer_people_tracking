### Ground-based RGB-D people detector from PCL

This ROS package is a wrapper around a slightly modified version of the ground-based RGB-D people detection module from the Point Cloud Library (PCL). It is based upon the implementation described in the following publication: 

> M. Munaro, F. Basso and E. Menegatti.  
> *Tracking people within groups with RGB-D data*  
> Proc. Int. Conf. on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.  

In contrast to the original implementation in PCL, the following features have been added in this version:
* Subscribe to an RGB-D stream via ROS, publish detections as spencer_tracking_msgs/DetectedPersons
* Publish extracted ROIs as visualization_msgs/MarkerArray for visualization in RViz
* Optionally use GPU-based HOG classifier from OpenCV library, instead of CPU version
* Transform input cloud from input coordinate frame such that the ground plane (given by its groundplane coefficients) is correctly aligned. This allows to use sensor setups in which the sensor is not perfectly horizontally aligned (e.g. positive/negative pitch), or even vertical sensor arrangements.



#### Credits

Original implementation in PCL by Matteo Munaro, IAS Lab, University of Padova.  
ROS wrapper for SPENCER by Timm Linder, Social Robotics Laboratory, University of Freiburg.  

/* Created on: Jun 23, 2014. Author: Fabian Girrbach */

#include <ros/ros.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <fstream>
#include <iostream>
#include <csignal>

std::ofstream file_gt_persons, file_gt_tracks, file_persons, file_tracks, file_compare_tracks, file_compare_detections;

void callback_gt_persons(const spencer_tracking_msgs::DetectedPersonsConstPtr &msg){
  if (file_gt_persons.is_open())
    {
      file_gt_persons << msg->header.stamp.toNSec() << " ; " << msg->detections.size();
      for (int i = 0; i < msg->detections.size(); i++){
          file_gt_persons << " ; " << msg->detections.at(i).detection_id;
        }
      file_gt_persons << std::endl;
    }

}

void callback_gt_tracks(const spencer_tracking_msgs::TrackedPersonsConstPtr &msg){
  if (file_gt_tracks.is_open())
    {
      file_gt_tracks << msg->header.stamp.toNSec() << " ; " << msg->tracks.size();
      for (int i = 0; i < msg->tracks.size(); i++){
            if (msg->tracks.at(i).pose.pose.position.x*msg->tracks.at(i).pose.pose.position.x +
                    msg->tracks.at(i).pose.pose.position.y*msg->tracks.at(i).pose.pose.position.y < 20*20)
            file_gt_tracks << " ; " << msg->tracks.at(i).track_id << " ; "  << msg->tracks.at(i).pose.pose.position.x  << " ; "  << msg->tracks.at(i).pose.pose.position.y  ;
        }
      file_gt_tracks << std::endl;
    }

}


void callback_persons(const spencer_tracking_msgs::DetectedPersonsConstPtr &msg){
  if (file_persons.is_open())
    {
      file_persons << msg->header.stamp.toNSec() << " ; " << msg->detections.size();
      for (int i = 0; i < msg->detections.size(); i++){
          file_persons << " ; " << msg->detections.at(i).detection_id;
        }
      file_persons << std::endl;
    }

}

void callback_tracks(const spencer_tracking_msgs::TrackedPersonsConstPtr &msg){
  if (file_tracks.is_open())
    {
      file_tracks << msg->header.stamp.toNSec() << " ; " << msg->tracks.size();
      for (int i = 0; i < msg->tracks.size(); i++){
            file_tracks << " ; " << msg->tracks.at(i).track_id << " ; " << msg->tracks.at(i).pose.pose.position.x  << " ; "  << msg->tracks.at(i).pose.pose.position.y  ; ;
        }
      file_tracks << std::endl;
    }

}



void signalHandler( int signum )
{
  ROS_INFO_STREAM("Interrupt signal (" << signum << ") received.\n");

  file_gt_persons.close();
  file_gt_tracks.close();
  file_persons.close();
  file_tracks.close();
  ROS_INFO_STREAM("Files were closed. " << std::endl);
  exit(signum);

}

double checkEuclideanDist(const geometry_msgs::PoseWithCovariance pose1, const geometry_msgs::PoseWithCovariance pose2)
{

  return sqrt((pose1.pose.position.x - pose2.pose.position.x)*(pose1.pose.position.x - pose2.pose.position.x) +
              (pose1.pose.position.y - pose2.pose.position.y)*(pose1.pose.position.y - pose2.pose.position.y));
}

void compare_tracks(const spencer_tracking_msgs::TrackedPersonsConstPtr &gt, const spencer_tracking_msgs::TrackedPersonsConstPtr &tracker){
  if (file_compare_tracks.is_open())
    {
      file_compare_tracks << gt->header.stamp.toNSec() << " ; " << gt->tracks.size() <<" ; " << tracker->tracks.size();
      for (int i = 0; i < tracker->tracks.size(); i++){
          double min_distance(100.0);
                for (int j = 0; j < gt->tracks.size(); j++)
                {
                      double distance = checkEuclideanDist(gt->tracks.at(j).pose, tracker->tracks.at(i).pose);
                      if (distance < min_distance){
                          min_distance = distance;
                      }
                }
              file_compare_tracks << " ; " << min_distance ;
            }
      file_compare_tracks << std::endl;

    }
}


void compare_detections(const spencer_tracking_msgs::DetectedPersonsConstPtr &gt, const spencer_tracking_msgs::DetectedPersonsConstPtr &tracker){
  if (file_compare_detections.is_open())
    {
      file_compare_detections << gt->header.stamp.toNSec() << " ; " << gt->detections.size() <<" ; " << tracker->detections.size();
      for (int i = 0; i < tracker->detections.size(); i++){
          double min_distance(100.0);
          for (int j = 0; j < gt->detections.size(); j++){
              double distance = checkEuclideanDist(gt->detections.at(j).pose, tracker->detections.at(i).pose);
              if (distance < min_distance){
                  min_distance = distance;
                }
            }
          file_compare_detections << " ; " << min_distance ;

        }
      file_compare_detections << std::endl;

    }
}





int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "tracking_logger");
  ros::NodeHandle n;


  std::signal(SIGINT, signalHandler);
  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(100));



  //Create Files
  file_gt_persons.open("gt_persons.log", std::ios::trunc);
  if (file_gt_persons.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_gt_persons << "timestamp ; number of detections ; detection_ids..." << std::endl;
    }
  file_gt_tracks.open("gt_tracks.log", std::ios::trunc);
  if (file_gt_tracks.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_gt_tracks << "timestamp ; number of tracks ; track_id ; pose_x ; pose_y ..." << std::endl;
    }
  file_persons.open("detector_persons.log", std::ios::trunc);
  if (file_persons.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_persons << "timestamp ; number of detections ; detection_ids..." << std::endl;
    }
  file_tracks.open("tracker_tracks.log", std::ios::trunc);
  if (file_tracks.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_tracks << "timestamp ; number of tracks ; track_id ; pose_x ; pose_y ..." << std::endl;
    }
  file_compare_tracks.open("tracker_comparison.log", std::ios::trunc);
  if (file_compare_tracks.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_compare_tracks << "timestamp ; number of tracks ground truth ; number of tracks tracker  ; min_distance track <-> gt ..." << std::endl;
    }
  file_compare_detections.open("detection_comparison.log", std::ios::trunc);
  if (file_compare_detections.is_open())
    {
      ROS_INFO_STREAM("File was created " << std::endl);
      file_compare_detections << "timestamp ; number of detections ground truth ; number of detections  ; min_distance detection <-> gt .." << std::endl;
    }

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  message_filters::Subscriber<spencer_tracking_msgs::DetectedPersons> sub_gt_persons(n,"/groundtruth/detected_persons", 1000);
  sub_gt_persons.registerCallback(boost::bind(&callback_gt_persons, _1));
  message_filters::Subscriber<spencer_tracking_msgs::TrackedPersons> sub_gt_tracks(n,"/groundtruth/tracked_persons", 1000);
  sub_gt_tracks.registerCallback(boost::bind(&callback_gt_tracks, _1));
  message_filters::Subscriber<spencer_tracking_msgs::DetectedPersons> sub_persons(n,"/spencer/perception/detected_persons", 1000);
  sub_persons.registerCallback(boost::bind(&callback_persons, _1));
  message_filters::Subscriber<spencer_tracking_msgs::TrackedPersons> sub_tracks(n,"/spencer/perception/tracked_persons", 1000);
  sub_tracks.registerCallback(boost::bind(&callback_tracks, _1));

  typedef message_filters::sync_policies::ApproximateTime<spencer_tracking_msgs::TrackedPersons, spencer_tracking_msgs::TrackedPersons> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy
  message_filters::Synchronizer<MySyncPolicy> sync_tracks(MySyncPolicy(100),sub_gt_tracks, sub_tracks);
  sync_tracks.registerCallback(boost::bind(&compare_tracks, _1, _2));
  message_filters::TimeSynchronizer<spencer_tracking_msgs::DetectedPersons, spencer_tracking_msgs::DetectedPersons> sync_detections(sub_gt_persons, sub_persons, 1);
  sync_detections.registerCallback(boost::bind(&compare_detections, _1, _2));



  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }


  return 0;
} // end main()





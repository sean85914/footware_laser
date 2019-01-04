#ifndef PIXEL_TO_WAYPOINT_H
#define PIXEL_TO_WAYPOINT_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
// ROS message types
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointXYZRGB;

const int EPSILON = 7; // 1-norm distance on neighborhood
// Topic string
const std::string PC_STR = "/camera/depth_registered/points"; // Pointcloud topic string
const std::string PUB_STR = "/waypoint_array";
const std::string SUB_STR = "/pixel_array";
// Frame string 
const std::string CAMERA_STR = "camera_rgb_optical_frame"; // Pointcloud frame id
const std::string BASE_STR = "base_link"; // Robot arm base link frame id

class Pixel2Waypoint
{
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher pub;
  geometry_msgs::PoseArray arr;
  PointXYZRGB pc_;
  bool valid = true, // Is the transformation from 2D to 3D is valid?
       has_tf; // Is the transform from [camera_rgb_optical_frame] -> [base_link] exist?
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Matrix3x3 rot_mat; // Rotation matrix from [camera_rgb_optical_frame] -> [base_link]
  tf::Vector3 trans; // Translation vector from [camera_rgb_optical_frame] -> [base_link]
  void getTransform(void); // Get transform, called when initialize
  void cb(const geometry_msgs::PoseArray msg); // Callback of subscriber
 public:
  Pixel2Waypoint(); // Constructor
};

#endif

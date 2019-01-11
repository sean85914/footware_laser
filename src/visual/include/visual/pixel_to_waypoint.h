#ifndef PIXEL_TO_WAYPOINT_H
#define PIXEL_TO_WAYPOINT_H

#include <algorithm> // std::rotate

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
// Eigen
#include <Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointXYZRGB;

const int EPSILON = 7; // 1-norm distance on neighborhood
// Topic string
const std::string PC_STR = "/camera/depth_registered/points"; // Pointcloud topic string
const std::string PUB_STR = "/waypoint_array"; // Publish topic string
const std::string SUB_STR = "/pixel_array"; // Subscribe topic string
// Frame string 
const std::string CAMERA_STR = "camera_rgb_optical_frame"; // Pointcloud frame id
const std::string BASE_STR = "base_link"; // Robot arm base link frame id
const std::string TCP_STR = "tcp_link"; // TCP link frame id

class Pixel2Waypoint
{
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher pub;
  geometry_msgs::PoseArray arr; // output to publish
  PointXYZRGB pc_;
  bool valid = true, // Is the transformation from 2D to 3D is valid?
       has_tf; // Is the transform from [base_link] -> [camera_rgb_optical_frame] exist?
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Matrix3x3 rot_mat; // Rotation matrix from [base_link] -> [camera_rgb_optical_frame]
  tf::Vector3 trans, // Translation vector from [base_link] -> [camera_rgb_optical_frame]
              x_vec, // TCP x-axis
              tcp_point; // TCP point w.r.t base_link
  void getTransform(void); // Get transform, called when initialize
  void cb(const geometry_msgs::PoseArray msg); // Callback of subscriber
  void reorder_array(void); // Re-order output pose array so that the first point is the nearest one
 public:
  Pixel2Waypoint(); // Constructor
};

#endif

#include <ros/ros.h>
#include <tf/transform_listener.h>
// ROS message types
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const int EPSILON = 7;
const std::string PC_STR = "/camera/depth_registered/points";
const std::string BASE_STR = "base_link";
const std::string CAMERA_STR = "camera_rgb_optical_frame";

class Pixel2Waypoint
{ 
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Publisher pub;
  geometry_msgs::PoseArray arr;
  pcl::PointCloud<pcl::PointXYZRGB> pc_;
  bool valid = true, has_tf;
  tf::TransformListener listener; 
  tf::StampedTransform transform;
  tf::Matrix3x3 rot_mat;
  tf::Vector3 trans;
  void getTransform()
  {
    try{
      listener.waitForTransform(BASE_STR, CAMERA_STR, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(BASE_STR, CAMERA_STR, ros::Time(0), transform);
      rot_mat = tf::Matrix3x3(transform.getRotation());
      trans = tf::Vector3(transform.getOrigin());
      has_tf = true;
    } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what()); has_tf = false;}
  }
  void cb(const geometry_msgs::PoseArray msg)
  {
    std::cout << "Receive data with " << msg.poses.size() << " points" << std::endl;
    sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>\
                                          (PC_STR, ros::Duration(5.0));
    getTransform();
    if(pc == NULL) {ROS_INFO("Not point cloud received.");}
    else{
      pcl::fromROSMsg(*pc, pc_);
      for(int idx=0; idx<msg.poses.size(); ++idx){
        double pixel_x = msg.poses[idx].position.x,
               pixel_y = msg.poses[idx].position.y;
        double wp_x = pc_.at(pixel_x, pixel_y).x,
               wp_y = pc_.at(pixel_x, pixel_y).y, 
               wp_z = pc_.at(pixel_x, pixel_y).z;
        if(std::isnan(wp_x)){
          // Handle nan
          double cali_x = 0, cali_y = 0, cali_z = 0;
          int count = 0;
          for(int i=pixel_x-EPSILON/2; i<=pixel_x+EPSILON/2; ++i){
            for(int j=pixel_y-EPSILON/2; j<=pixel_y+EPSILON/2; ++j){
              if(!std::isnan(pc_.at(i, j).x)){
                ++count; 
                cali_x += pc_.at(i, j).x;
                cali_y += pc_.at(i, j).y;
                cali_z += pc_.at(i, j).z;
              } // end if
            } // end for (j)
          } // end for (i)
          if(count!=0){
            valid = true;
            wp_x = cali_x/(float)count;
            wp_y = cali_y/(float)count;
            wp_z = cali_z/(float)count;
          } else{
            std::cout << "Pixel index: " << idx << " with 1-norm distance " << EPSILON/2 
                      << " neighborhood still get nan, ignoring..." << std::endl;
            valid = false;
          } // end else
        } // end if
        if(valid and has_tf){
          // Transform to base_link frame
          geometry_msgs::Pose wp;
          tf::Vector3 vec     = tf::Vector3(wp_x, wp_y, wp_z),
                      vec_rot = rot_mat * vec,
                      vec_tf  = vec_rot + trans; 
          wp.position.x = vec_tf.x(); wp.position.y = vec_tf.y(); wp.position.z = vec_tf.z();
          arr.poses.push_back(wp);
        } // end if
      } // end for (idx)
      std::cout << "After processing, there are " << arr.poses.size() << " waypoints." << std::endl;
      pub.publish(arr);
      ros::shutdown();
    } // end else
  }
 public:
  Pixel2Waypoint(){
    pub = nh_.advertise<geometry_msgs::PoseArray>("/waypoint_array", 1);
    sub = nh_.subscribe("/pixel_array", 1, &Pixel2Waypoint::cb, this);
  }  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "convert_to_waypoints");
  Pixel2Waypoint n;
  while(ros::ok()) {ros::spin();}
  return 0;
}

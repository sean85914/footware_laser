#include <cstdlib>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <arm_operation/target_pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

const double x = 0.029; 
const double y = -0.694;
const double z = 0.392;
bool done = false; // Is callback done?

geometry_msgs::PoseArray pose_array;
// Get orientation from work point to target
// Concept: fix tool center point, point toward our target point, that is, x vector will be start
//          from TCP to our target point, use Z-axis of base link to cross this vector to yell our 
//          Z-axis and form our orientation
// Params:
//   x_t: target point x
//   y_t: target point y
//   z_t: target point z
//   target: reference to target_pose
void get_target_orientation(double x_t, double y_t, double z_t, arm_operation::target_pose &target)
{
  double dx = x_t - x,
         dy = y_t - y,
         dz = z_t - z;
  Eigen::Vector3d x_v(dx, dy, dz),
                  z_v(-dy, dx, 0);
  Eigen::Vector3d x_hat = x_v.normalized(),
                  z_hat = z_v.normalized(),
                  y_hat = z_hat.cross(x_hat);
  tf::Matrix3x3 rot_mat(x_hat[0], y_hat[0], z_hat[0],
                        x_hat[1], y_hat[1], z_hat[1],
                        x_hat[2], y_hat[2], z_hat[2]);
  tf::Quaternion quat;
  rot_mat.getRotation(quat);
  ROS_INFO("Quaternion: %f %f %f %f", quat.getX(), quat.getY(), quat.getZ(), quat.getW());
  target.request.target_pose.position.x = x;
  target.request.target_pose.position.y = y;
  target.request.target_pose.position.z = z;
  target.request.target_pose.orientation.x = quat.getX();
  target.request.target_pose.orientation.y = quat.getY();
  target.request.target_pose.orientation.z = quat.getZ();
  target.request.target_pose.orientation.w = quat.getW();
}
// Callback for /waypoint_array
// Receive the array and let robot arm to follow the trajectory
void cb_array(const geometry_msgs::PoseArray msg)
{
  if(!done){
    std::cout << "Receive data, processing..." << std::endl;
    pose_array = msg;
    std::cout << "There are total " << pose_array.poses.size() << " waypoints." << std::endl;
    done = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shoes_edge_tracker");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/waypoint_array", 10, cb_array);
  ros::ServiceClient goto_pose_client;
  goto_pose_client = nh.serviceClient<arm_operation::target_pose>("/ur5_control/goto_pose");
  arm_operation::target_pose target;
  target.request.factor = 0.7;
  while(!done and ros::ok()) {ros::spinOnce();} // Wait callback process terminated
  for(int i=0; i<pose_array.poses.size(); ++i){
    double x_t = pose_array.poses[i].position.x,
           y_t = pose_array.poses[i].position.y,
           z_t = pose_array.poses[i].position.z;
    get_target_orientation(x_t, y_t, z_t, target);
    goto_pose_client.call(target);
    std::cout << "Finish waypoint index " << i << std::endl;
    ros::Duration(0.5).sleep();
  }
  return 0;
}

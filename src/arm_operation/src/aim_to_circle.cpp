#include <cstdlib>
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <arm_operation/target_pose.h>
#include <math.h>
#include <cmath> 
#define deg2rad(x) (x*M_PI/180.0)

//手臂末端工作點 
const double x = 0.029; 
const double y = -0.694;
const double z = 0.392;
const double radius = 0.05;
const int N = 50; 



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
  //ROS_INFO("Quaternion: %f %f %f %f", quat.getX(), quat.getY(), quat.getZ(), quat.getW());
  target.request.target_pose.position.x = x;
  target.request.target_pose.position.y = y;
  target.request.target_pose.position.z = z;
  target.request.target_pose.orientation.x = quat.getX();
  target.request.target_pose.orientation.y = quat.getY();
  target.request.target_pose.orientation.z = quat.getZ();
  target.request.target_pose.orientation.w = quat.getW();
}

int main(int argc, char** argv)
{
  if(argc!=4) {
    std::cout << "Incorrect number of input arguments" << std::endl;
    return -1;
  }
  ros::init(argc, argv, "aim_to_target");
  ros::NodeHandle nh;
  ros::ServiceClient goto_pose_client;
  goto_pose_client = nh.serviceClient<arm_operation::target_pose>("/ur5_control/goto_pose");
  arm_operation::target_pose target;
  target.request.factor = 0.7;
  double x_t = atof(argv[1]),
         y_t = atof(argv[2]),
         z_t = atof(argv[3]),
         theta = 0;
  for(int i=0; i<=N; i++)
  {
    std::cout << "Index: " << i << std::endl;
  	get_target_orientation(x_t+radius*sin(theta), y_t+radius*cos(theta), z_t, target); 
  	goto_pose_client.call(target);
    theta += 2*M_PI*1/N;
    ros::Duration(0.5).sleep();
  }

  return 0;
}

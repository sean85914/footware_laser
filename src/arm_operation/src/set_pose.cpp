#include <ros/ros.h>
#include <ros/package.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ur_kin.h>

#include <cmath>
#include <fstream>

// Read the waypoints file and make UR5 traverse through it
// Input: file name
// Output: UR5 traverse the waypoints from the first to the last one

// Last modify: 9/11
// Editor: Sean

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  TrajClient* traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal_;  
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_state_;
  ros::ServiceClient client_;

  double joint_[6];
  double position_now[6];

  void jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < 6; ++i)
      joint_[i] = msg.position[i];
  }
  
 public:
  RobotArm() : joint_() {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/follow_joint_trajectory", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0))) 
      ROS_INFO("Waiting for the joint_trajectory_action server");
    

    // Subscribe to /ariac/joint_state
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &RobotArm::jointStateCallback, this);
    
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.joint_names.resize(6);
    t.joint_names[0] = "shoulder_pan_joint";
    t.joint_names[1] = "shoulder_lift_joint";
    t.joint_names[2] = "elbow_joint";
    t.joint_names[3] = "wrist_1_joint";
    t.joint_names[4] = "wrist_2_joint";
    t.joint_names[5] = "wrist_3_joint";
  }
  ~RobotArm() {
    delete traj_client_;
  }
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  } 

  control_msgs::FollowJointTrajectoryGoal armToDesiredJointStateTrajectory(double* js) {
    ros::spinOnce();
 
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2); 
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);
    }

    t.points[0].time_from_start = ros::Duration(0);
    // Calculate for duration, based on joint space distance
    double dist = 0, cost_time;
    for(int i=0;i<6;++i){
    	dist += pow(joint_[i] - js[i], 2);
    }
    dist = sqrt(dist);
    cost_time = ceil(dist/.5);
    t.points[1].time_from_start = ros::Duration(cost_time);

    for (int i = 0; i < 6; ++i) { 
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = js[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }

  actionlib::SimpleClientGoalState getState() {
    ros::spinOnce();
    return traj_client_->getState();
  }
};

double** parse_file(std::string file_str, int *list_len)
{
    std::string line;
    std::ifstream fp;
    int counter = 0;
    int index = 0;
    double** res = 0;
    fp.open(file_str.c_str(), std::ifstream::in);
    if(!fp){
        ROS_ERROR("Can't open file.");
        return NULL;
    }
    while(std::getline(fp, line)){
        ++counter;
    }
    *list_len = counter;
    res = new double*[counter];
    for(int i=0; i<counter; ++i){
        res[i] = new double[6];
    }
    // return the cursor to the begining of the file
    fp.clear();
    fp.seekg(0, std::ios::beg);
    while(!fp.eof()){
        std::getline(fp, line);
        std::stringstream ss(line);
        int i = 0;
        while(ss.good() && i < 6){
            ss >> res[index][i];
            ++i;
        }
        ++index;
    }
    // Close the file
    fp.close();
    return res;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_pose");
    ros::NodeHandle nh;
    RobotArm arm;
    ros::Time now = ros::Time::now();

    ROS_INFO("Initializing...");
    while (ros::Time::now().toSec() < (now.toSec() + 5)) {
      ros::spinOnce();
    }

    ROS_INFO("Reading input file...");
    std::string file_name, file_str, path = ros::package::getPath("arm_operation");
    int counter;
    ros::param::get("~file_name", file_name);
    file_str = path + "/data/" + file_name + ".txt";
    double** waypoints = parse_file(file_str, &counter);
    if(!waypoints) {return 0;}

    ROS_INFO("There are %d waypoints", counter);
    ROS_INFO("Start to traverse all waypoints...");
    for(int index_togo = 0; index_togo < counter; ++index_togo){
        arm.startTrajectory(arm.armToDesiredJointStateTrajectory(waypoints[index_togo]));
        while(!arm.getState().isDone() && ros::ok()) {
            usleep(500000);
        }
        ROS_INFO("Waypoint index: %d", index_togo);
    }
    return 0;
}

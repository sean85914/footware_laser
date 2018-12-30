#include "ros/ros.h"
#include <cstdlib>
#include <arm_operation/target_pose.h>

int main(int argc, char **argv)
{
    if (argc <8) {std::cout << "Not enough input" << std::endl; return -1;}
	ros::init(argc, argv, "ur5_control"); 
	
	ros::NodeHandle nh;
	
	//for(int i=0; i<4; i++)
	//{
		ros::ServiceClient goto_pose_client;
		goto_pose_client = nh.serviceClient<arm_operation::target_pose>("/ur5_control/goto_pose");
		
		arm_operation::target_pose srv;
		srv.request.target_pose.position.x = atof(argv[1]);
		srv.request.target_pose.position.y = atof(argv[2]);
		srv.request.target_pose.position.z = atof(argv[3]);
		srv.request.target_pose.orientation.x = atof(argv[4]);
		srv.request.target_pose.orientation.y = atof(argv[5]);
		srv.request.target_pose.orientation.z = atof(argv[6]);
		srv.request.target_pose.orientation.w = atof(argv[7]);
		
		if (goto_pose_client.call(srv))
		{
			ROS_INFO("Goto pose goal  %f %f %f %f %f %f %f", srv.request.target_pose.position.x,\
	                                                         srv.request.target_pose.position.y,\
	                                                         srv.request.target_pose.position.z,\
	                                                         srv.request.target_pose.orientation.x,\
	                                                         srv.request.target_pose.orientation.y,\
	                                                         srv.request.target_pose.orientation.z,\
	                                                         srv.request.target_pose.orientation.w);
		}
		else
		{
			ROS_ERROR("Failed to call service ur5_control/goto_pose");
			return 1;
		}	
	//}
	
	
	return 0;
}

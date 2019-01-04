#include <visual/pixel_to_waypoint.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pixel_to_waypoint");
  Pixel2Waypoint p2w;
  while(ros::ok()) {ros::spin();}
  return 0;
}

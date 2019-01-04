#include <visual/pixel_to_waypoint.h>

Pixel2Waypoint::Pixel2Waypoint()
{
  pub = nh_.advertise<geometry_msgs::PoseArray>(PUB_STR, 1);
  sub = nh_.subscribe(SUB_STR, 1, &Pixel2Waypoint::cb, this);
  getTransform();
  if(has_tf) ROS_INFO("Get transformation from [%s] to [%s]", CAMERA_STR.c_str(), BASE_STR.c_str());
  else ROS_ERROR("Fail to get transformation from [%s] to [%s]", CAMERA_STR.c_str(), BASE_STR.c_str());
}

void Pixel2Waypoint::getTransform(void)
{
  try{
    listener.waitForTransform(BASE_STR, CAMERA_STR, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(BASE_STR, CAMERA_STR, ros::Time(0), transform);
    rot_mat = tf::Matrix3x3(transform.getRotation());
    trans = tf::Vector3(transform.getOrigin());
    has_tf = true;
  } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what()); has_tf = false;}
}

void Pixel2Waypoint::cb(const geometry_msgs::PoseArray msg)
{
  ROS_INFO("Receive data with %d points", (int)msg.poses.size());
  sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>\
                                        (PC_STR, ros::Duration(5.0));
  if(pc == NULL) {ROS_ERROR("Not point cloud received."); return;}
  pcl::fromROSMsg(*pc, pc_);
  for(int idx=0; idx<msg.poses.size(); ++idx){
    int pixel_x = msg.poses[idx].position.x,
           pixel_y = msg.poses[idx].position.y;
    double wp_x = pc_.at(pixel_x, pixel_y).x,
           wp_y = pc_.at(pixel_x, pixel_y).y,
           wp_z = pc_.at(pixel_x, pixel_y).z;
    if(std::isnan(wp_x)){ // Handle nan
      double cali_x = 0, cali_y = 0, cali_z = 0;
      int count = 0;
      for(int x_=pixel_x-EPSILON/2; x_<=pixel_x+EPSILON/2; ++x_){
        if(x_<0 or x_>=pc_.width) {ROS_ERROR("Reach width edge, ignore..."); continue;}
        for(int y_=pixel_y-EPSILON/2; y_<=pixel_y+EPSILON/2; ++y_){
          if(y_<0 or y_>=pc_.height) {ROS_ERROR("Reach height edge, ignore..."); continue;}
          if(!std::isnan(pc_.at(x_,y_).x)){
            ++count;
            cali_x += pc_.at(x_, y_).x; cali_y += pc_.at(x_, y_).y; cali_z += pc_.at(x_, y_).z;
          } // end if (!std::isnan(pc_.at(x_, y_).x))
        } // end for (y_)
      } // end for (x_)
      if(count!=0){
        valid = true; // The result is valid
        wp_x = cali_x/count; wp_y = cali_y/count; wp_z = cali_z/count;
      } // end if (count!=0)
      else {
        ROS_ERROR("Pixel: (%d, %d) with 1-norm distance %d neighborhood still get nan, ignoring...",
                 pixel_x, pixel_y, EPSILON/2);
        valid = false; // Invalid result
      } // end else
    } // end if (std::isnan(wp_x))
    if(valid and has_tf){ // Transform to base_link frame
      tf::Vector3 vec     = tf::Vector3(wp_x, wp_y, wp_z),
                  vec_rot = rot_mat*vec, // Rotate first
                  vec_tf  = vec_rot + trans; // Then add translation
      geometry_msgs::Pose wp;
      wp.position.x = vec_tf.x(); wp.position.y = vec_tf.y(); wp.position.z = vec_tf.z();
      arr.poses.push_back(wp);
    } // end if(valid and has_tf)
  } // end for (idx)
  ROS_INFO("After processing, there are %d waypoints.", (int)arr.poses.size());
  pub.publish(arr);
  ros::shutdown(); // Only process once;
}

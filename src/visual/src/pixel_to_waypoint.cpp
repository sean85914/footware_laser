#include <visual/pixel_to_waypoint.h>

Pixel2Waypoint::Pixel2Waypoint()
{
  pub = nh_.advertise<geometry_msgs::PoseArray>(PUB_STR, 1);
  sub = nh_.subscribe(SUB_STR, 1, &Pixel2Waypoint::cb, this);
  getTransform();
  if(has_tf) ROS_INFO("Get transformation from [%s] to [%s]", BASE_STR.c_str(), CAMERA_STR.c_str());
  else ROS_ERROR("Fail to get transformation from [%s] to [%s]", \
                 BASE_STR.c_str(), CAMERA_STR.c_str());
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
  reorder_array();
  pub.publish(arr);
  ros::shutdown(); // Only process once;
}

void Pixel2Waypoint::reorder_array(void)
{
  // Least-square plane fitting
  Eigen::MatrixXd A((int)arr.poses.size(), 3),
                  x(3, 1),
                  b((int)arr.poses.size(), 1);
  // Build matrix A and b
  for(int i=0; i<arr.poses.size(); ++i){
    A(i, 0) = arr.poses[i].position.x;
    A(i, 1) = arr.poses[i].position.y;
    A(i, 2) = -1;
    b(i, 0) = -arr.poses[i].position.z;
  } // end for (i)
  // x = (A'A)^-1*b
  x = (A.transpose()*A).inverse()*b;
  // Get the transform from [base_link] to [tcp_link]
  try{
    listener.waitForTransform(BASE_STR, TCP_STR, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(BASE_STR, TCP_STR, ros::Time(0), transform);
    x_vec = transform.getBasis().getColumn(0);
    tcp_point = transform.getOrigin();
  } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what());}
  // Get the projection point on the plane determined above
  double p_a = x(0, 0), p_b = x(0, 1), p_c = x(0, 2), // Plane parameters
         x_0 = tcp_point.getX(), y_0 = tcp_point.getY(), z_0 = tcp_point.getZ(), // TCP point
         x_x = x_vec.getX(), x_y = x_vec.getY(), x_z = x_vec.getZ(), // x-axis vector
         t_bar = (p_c-p_a*x_0-p_b*y_0-z_0)/(p_a*x_x+p_b*x_y+x_z); 
         // Parameter value for line equation
  double x_p = x_0 + x_x * t_bar, 
         y_p = y_0 + x_y * t_bar, 
         z_p = z_0 + x_z * t_bar; // Projection point on plane
  double min_dis = 1e6;
  int min_idx = -1;
  // Calculate distance between each point to projection point and get the minimum index
  for(int i=0; i<arr.poses.size(); ++i) {
    double dx = x_p - arr.poses[i].position.x,
           dy = y_p - arr.poses[i].position.y, 
           dz = z_p - arr.poses[i].position.z,
           dis_2 = dx*dx + dy*dy + dz*dz; // distance square
    if(dis_2 < min_dis){
      min_dis = dis_2; min_idx = i;
    } // end if(dis_2 < min_dis)
  } // end for (i)
  // Circular shift toward right with min_idx
  std::rotate(arr.poses.begin(), arr.poses.begin()+((int)arr.poses.size()-min_idx), arr.poses.end());
}

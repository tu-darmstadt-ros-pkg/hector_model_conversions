

#include "mesh_to_sampled_point_cloud/mesh_to_sampled_point_cloud.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mesh_to_sampled_point_cloud::MeshToSampledPointCloud mesh_to_sampled_point_cloud_converter(nh, pnh);

  ros::spin();


  return 0;
}


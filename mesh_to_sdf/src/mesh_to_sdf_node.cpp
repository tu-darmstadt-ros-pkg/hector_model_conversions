

#include "mesh_to_sdf/mesh_to_sdf.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mesh_to_sdf::MeshToSdf mesh_to_sdf_converter(nh, pnh);

  ros::spin();


  return 0;
}

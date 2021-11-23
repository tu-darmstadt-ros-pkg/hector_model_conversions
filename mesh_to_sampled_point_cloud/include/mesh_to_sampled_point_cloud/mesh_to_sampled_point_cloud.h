

#ifndef MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H
#define MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H

#include "mesh_to_sampled_point_cloud/ConvertMeshToSampledPointCloud.h"
#include "mesh_to_sampled_point_cloud/pcl_mesh_sampling_extract.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace mesh_to_sampled_point_cloud
{

class MeshToSampledPointCloud
{
public:

  MeshToSampledPointCloud(ros::NodeHandle& nh, ros::NodeHandle& pnh);


private:

  bool
  convertMeshToSampledPointCloudCallback(mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Request& request,
                                         mesh_to_sampled_point_cloud::ConvertMeshToSampledPointCloud::Response& response);


  /**
   * Convert a translation, rotation and scale_factor into a vtkTransform.
   * @param translation
   * @param rotation_euler_deg Euler angles (Z''*Y'*X) in degrees. Will be applied as rotateX, rotateY, rotateZ of vtkTransform.
   * @param scale_factor
   * @return
   */
  static vtkSmartPointer<vtkTransform>
  convertTransform(geometry_msgs::Vector3 translation, geometry_msgs::Vector3 rotation_euler_deg, double scale_factor);


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceServer convert_mesh_to_sampled_point_cloud_service_server_;
};
} // end namespace mesh_to_sampled_point_cloud


#endif //MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H

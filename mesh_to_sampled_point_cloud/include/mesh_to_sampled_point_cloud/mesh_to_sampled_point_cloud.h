

#ifndef MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H
#define MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H

#include "mesh_to_sampled_point_cloud/pcl_mesh_sampling_extract.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace mesh_to_sampled_point_cloud
{

class MeshToSampledPointCloud
{
public:

  /**
   * Import input_file and convert it to a point cloud using sampling.
   * @param input_file
   * @param num_sample_points
   * @param voxel_filter_leaf_size
   * @param translation
   * @param rotation_euler_deg
   * @param scale_factor
   * @param write_normals
   * @param write_colors
   */
  MeshToSampledPointCloud(const std::string& input_file, int num_sample_points, float voxel_filter_leaf_size,
                          geometry_msgs::Vector3 translation,
                          geometry_msgs::Vector3 rotation_euler_deg,
                          float scale_factor, bool write_normals, bool write_colors);

  /**
   * Get the sampled point cloud with data according to write_normals and write_colors, converted to a PCLPointCloud2.
   * @return sampled point cloud
   */
  std::shared_ptr<pcl::PCLPointCloud2> getPointCloud();

  /**
   * Save the point cloud to a pcd file.
   * @param output_file
   */
  void savePointCloudToPCDFile(const std::string& output_file);


private:

  /**
   * Convert a translation, rotation and scale_factor into a vtkTransform.
   * @param translation
   * @param rotation_euler_deg Euler angles (Z''*Y'*X) in degrees. Will be applied as rotateX, rotateY, rotateZ of vtkTransform.
   * @param scale_factor
   * @return
   */
  static vtkSmartPointer<vtkTransform>
  convertTransform(geometry_msgs::Vector3 translation, geometry_msgs::Vector3 rotation_euler_deg, double scale_factor);

  PCLMeshSamplingExtract mesh_sampling_extract_;
};
} // end namespace mesh_to_sampled_point_cloud


#endif //MESH_TO_SAMPLED_POINTCLOUD_MESH_TO_SAMPLED_POINT_CLOUD_H
